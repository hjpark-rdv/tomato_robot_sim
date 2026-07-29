import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException

from rbpodo_tomato_harvest.harvest_planner import CartesianHarvestPlanner


PLAN_RESULT_PREFIX = "__HARVEST_PLAN_RESULT__"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianHarvestPlanner()
    exit_code = 1
    try:
        trajectory = node.plan()
        execute_requested = bool(node.get_parameter("execute").value)
        execution_attempted = trajectory is not None and execute_requested
        execution_started = time.monotonic()
        success = trajectory is not None and node.execute(trajectory)
        execution_duration = (
            time.monotonic() - execution_started
            if execution_attempted
            else 0.0
        )
        node.last_plan_report["execution_requested"] = execute_requested
        node.last_plan_report["execution_attempted"] = execution_attempted
        node.last_plan_report["execution_success"] = (
            bool(success) if execution_attempted else None
        )
        node.last_plan_report["execution_duration_sec"] = round(
            execution_duration,
            6,
        )
        if execution_attempted and not success:
            node.last_plan_report["failure_stage"] = "EXECUTION"
            node.last_plan_report["failure_planner_type"] = "execution"
            node.last_plan_report[
                "failure_reason"
            ] = "TRAJECTORY_EXECUTION_FAILED"
        if success:
            exit_code = 0
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # Keep test failures visible in ros2 run output.
        node.get_logger().error(f"Tomato harvest test failed: {exc!r}")
    finally:
        print(
            f"{PLAN_RESULT_PREFIX}{json.dumps(node.last_plan_report)}",
            flush=True,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
