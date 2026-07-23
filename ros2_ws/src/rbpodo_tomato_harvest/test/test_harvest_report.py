import json

from rbpodo_tomato_harvest.harvest_report import write_report


def test_write_report_summarizes_failures_and_successes(tmp_path):
    session = {
        "created_at": "2026-07-23T12:00:00+09:00",
        "pipeline": "ompl",
        "planner_id": "RRTConnectkConfigDefault",
        "preapproach_mode": "planner",
    }
    results = [
        {
            "case": 1,
            "tomato": 0,
            "x": 0.3,
            "y": -0.2,
            "z": 0.4,
            "rotation_deg": 15.0,
            "success": False,
            "failure_stage": "CARTESIAN_PREAPPROACH",
            "failure_reason": "CARTESIAN_FRACTION_LOW",
            "cartesian_fraction": 0.8,
            "duration_sec": 0.5,
        },
        {
            "case": 1,
            "tomato": 1,
            "x": 0.3,
            "y": -0.2,
            "z": 0.4,
            "rotation_deg": 15.0,
            "success": True,
            "duration_sec": 0.3,
        },
    ]
    (tmp_path / "session.json").write_text(
        json.dumps(session),
        encoding="utf-8",
    )
    (tmp_path / "results.jsonl").write_text(
        "".join(json.dumps(result) + "\n" for result in results),
        encoding="utf-8",
    )

    report_path = write_report(tmp_path)
    report = report_path.read_text(encoding="utf-8")

    assert report_path.name == "report.html"
    assert "성공률" in report
    assert "50.0%" in report
    assert "CARTESIAN_PREAPPROACH" in report
    assert "CARTESIAN_FRACTION_LOW" in report
    assert "Pre-approach: planner" in report
