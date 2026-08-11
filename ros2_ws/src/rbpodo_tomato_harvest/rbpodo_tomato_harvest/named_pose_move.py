import json

import rclpy
from rclpy.executors import ExternalShutdownException

from rbpodo_tomato_harvest.harvest_planner import CartesianHarvestPlanner


NAMED_POSE_RESULT_PREFIX = "__NAMED_POSE_RESULT__"


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianHarvestPlanner()
    exit_code = 1
    try:
        if node.plan_and_execute_named_state():
            exit_code = 0
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:  # Keep command failures visible in GUI output.
        node.get_logger().error(f"Named pose Plan & Execute failed: {exc!r}")
        node.last_plan_report.setdefault("failure_stage", "NAMED_POSE_PROCESS")
        node.last_plan_report.setdefault("failure_planner_type", "process")
        node.last_plan_report.setdefault("failure_reason", repr(exc))
    finally:
        print(
            f"{NAMED_POSE_RESULT_PREFIX}"
            f"{json.dumps(node.last_plan_report)}",
            flush=True,
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
