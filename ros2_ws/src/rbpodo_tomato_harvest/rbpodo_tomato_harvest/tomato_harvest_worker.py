import json
import sys
import time

import rclpy
from rclpy.parameter import Parameter

from rbpodo_tomato_harvest.harvest_planner import CartesianHarvestPlanner


RESULT_PREFIX = "__HARVEST_RESULT__"


def _apply_request(node: CartesianHarvestPlanner, request: dict) -> None:
    tomato_frame = str(request["tomato_frame"])
    parameters = [
        Parameter("tomato_frame", value=tomato_frame),
        Parameter(
            "planning_pipeline_id",
            value=str(request.get("planning_pipeline_id", "ompl")),
        ),
        Parameter(
            "planner_id",
            value=str(request.get("planner_id", "RRTConnect")),
        ),
        Parameter(
            "preapproach_mode",
            value=str(request.get("preapproach_mode", "cartesian")),
        ),
        Parameter("execute", value=bool(request.get("execute", False))),
        Parameter(
            "pick_ready_velocity_scale",
            value=float(request.get("velocity_scale", 0.20)),
        ),
        Parameter(
            "pick_ready_acceleration_scale",
            value=float(request.get("acceleration_scale", 0.20)),
        ),
        Parameter(
            "harvest_wait_sec",
            value=float(request.get("harvest_wait_sec", 2.0)),
        ),
        Parameter("publish_display_trajectory", value=False),
    ]
    results = node.set_parameters(parameters)
    failures = [result.reason for result in results if not result.successful]
    if failures:
        raise RuntimeError("; ".join(failures))
    # The planner caches frame names that define its static robot setup. Only the
    # tomato target changes between requests in this persistent worker.
    node.tomato_frame = tomato_frame


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CartesianHarvestPlanner()
    try:
        for line in sys.stdin:
            request = {}
            node.last_plan_report = {}
            try:
                request = json.loads(line)
                if request.get("command") == "quit":
                    break
                request_id = int(request["request_id"])
                _apply_request(node, request)
                plan = node.plan()
                execute_requested = bool(request.get("execute", False))
                execution_attempted = plan is not None and execute_requested
                execution_started = time.monotonic()
                success = plan is not None and node.execute(plan)
                execution_duration = (
                    time.monotonic() - execution_started
                    if execution_attempted
                    else 0.0
                )
                node.last_plan_report["execution_requested"] = (
                    execute_requested
                )
                node.last_plan_report["execution_attempted"] = (
                    execution_attempted
                )
                node.last_plan_report["execution_success"] = (
                    bool(success) if execution_attempted else None
                )
                node.last_plan_report["execution_duration_sec"] = round(
                    execution_duration,
                    6,
                )
                if execution_attempted and not success:
                    node.last_plan_report["failure_stage"] = "EXECUTION"
                    node.last_plan_report[
                        "failure_planner_type"
                    ] = "execution"
                    node.last_plan_report[
                        "failure_reason"
                    ] = "TRAJECTORY_EXECUTION_FAILED"
                result = {
                    "request_id": request_id,
                    "success": bool(success),
                    "report": node.last_plan_report,
                }
            except Exception as error:
                result = {
                    "request_id": request.get("request_id", -1)
                    if isinstance(request, dict)
                    else -1,
                    "success": False,
                    "error": repr(error),
                    "report": node.last_plan_report,
                }
                node.get_logger().error(f"Persistent harvest request failed: {error!r}")
            print(f"{RESULT_PREFIX}{json.dumps(result)}", flush=True)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
