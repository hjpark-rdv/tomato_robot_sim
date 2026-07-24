import json

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
        if trajectory is not None and node.execute(trajectory):
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
