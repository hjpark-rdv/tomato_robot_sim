import argparse
import csv
from pathlib import Path

from tomato_moveit_teach.trajectory_dataset import (
    append_jsonl,
    rank_candidates,
    read_json,
    read_jsonl,
    train_logistic_ranker,
    weak_label_samples_from_diagnostics,
    write_json,
)


def cmd_extract_weak_labels(args) -> int:
    diagnostics = read_jsonl(args.diagnostics)
    samples = weak_label_samples_from_diagnostics(
        diagnostics,
        good_fraction=args.good_fraction,
        bad_fraction=args.bad_fraction,
        metric=args.metric,
    )
    output = Path(args.output).expanduser()
    if output.exists() and not args.append:
        output.unlink()
    for sample in samples:
        append_jsonl(output, sample)
    print(f"wrote weak-label samples: {len(samples)} -> {output}")
    return 0


def cmd_train(args) -> int:
    samples = read_jsonl(args.dataset)
    model = train_logistic_ranker(
        samples,
        learning_rate=args.learning_rate,
        epochs=args.epochs,
        l2=args.l2,
    )
    write_json(args.model_out, model)
    training = model["training"]
    print(
        "trained trajectory selector "
        f"samples={training['sample_count']} "
        f"positive={training['positive_count']} "
        f"negative={training['negative_count']} "
        f"accuracy={training['final_accuracy']:.3f} "
        f"-> {Path(args.model_out).expanduser()}"
    )
    return 0


def cmd_rank(args) -> int:
    model = read_json(args.model)
    diagnostics = read_jsonl(args.diagnostics)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    csv_output = Path(args.csv_output).expanduser() if args.csv_output else None
    if csv_output:
        csv_output.parent.mkdir(parents=True, exist_ok=True)

    rows = []
    if output.exists():
        output.unlink()
    for payload_index, payload in enumerate(diagnostics):
        ranked = rank_candidates(payload, model)
        for item in ranked:
            item["diagnostics_index"] = payload_index
            append_jsonl(output, item)
            rows.append(item)

    if csv_output:
        with csv_output.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(
                stream,
                fieldnames=[
                    "diagnostics_index",
                    "learned_rank",
                    "source_rank",
                    "learned_good_probability",
                    "trajectory_duration_sec",
                    "trajectory_score",
                    "gripper_spin_deg",
                    "gripper_x_sign",
                ],
            )
            writer.writeheader()
            for item in rows:
                writer.writerow(
                    {
                        "diagnostics_index": item["diagnostics_index"],
                        "learned_rank": item["learned_rank"],
                        "source_rank": item["source_rank"],
                        "learned_good_probability": f"{float(item['learned_good_probability']):.6f}",
                        "trajectory_duration_sec": f"{float(item['trajectory_duration_sec']):.6f}",
                        "trajectory_score": f"{float(item['trajectory_score']):.6f}",
                        "gripper_spin_deg": f"{float(item['gripper_spin_deg']):.3f}",
                        "gripper_x_sign": f"{float(item['gripper_x_sign']):+.0f}",
                    }
                )

    if rows:
        best = rows[0]
        print(
            "ranked candidates "
            f"count={len(rows)} best_source_rank={best['source_rank']} "
            f"prob={float(best['learned_good_probability']):.3f} "
            f"duration={float(best['trajectory_duration_sec']):.3f}s "
            f"-> {output}"
        )
    else:
        print(f"ranked candidates count=0 -> {output}")
    return 0


def cmd_demo(args) -> int:
    output_dir = Path(args.output_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    dataset = output_dir / "weak_labels.jsonl"
    model_path = output_dir / "trajectory_selector_model.json"
    ranked_path = output_dir / "ranked_candidates.jsonl"
    ranked_csv = output_dir / "ranked_candidates.csv"

    extract_args = argparse.Namespace(
        diagnostics=args.diagnostics,
        output=dataset,
        good_fraction=args.good_fraction,
        bad_fraction=args.bad_fraction,
        metric=args.metric,
        append=False,
    )
    cmd_extract_weak_labels(extract_args)
    train_args = argparse.Namespace(
        dataset=dataset,
        model_out=model_path,
        learning_rate=args.learning_rate,
        epochs=args.epochs,
        l2=args.l2,
    )
    cmd_train(train_args)
    rank_args = argparse.Namespace(
        diagnostics=args.diagnostics,
        model=model_path,
        output=ranked_path,
        csv_output=ranked_csv,
    )
    cmd_rank(rank_args)
    print(f"demo complete: {output_dir}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Train/apply a learned tomato trajectory selector.")
    subparsers = parser.add_subparsers(dest="command", required=True)

    extract = subparsers.add_parser("extract-weak-labels")
    extract.add_argument("--diagnostics", required=True)
    extract.add_argument("--output", required=True)
    extract.add_argument("--metric", default="trajectory_duration_sec")
    extract.add_argument("--good-fraction", type=float, default=0.25)
    extract.add_argument("--bad-fraction", type=float, default=0.25)
    extract.add_argument("--append", action="store_true")
    extract.set_defaults(func=cmd_extract_weak_labels)

    train = subparsers.add_parser("train")
    train.add_argument("--dataset", required=True)
    train.add_argument("--model-out", required=True)
    train.add_argument("--learning-rate", type=float, default=0.08)
    train.add_argument("--epochs", type=int, default=600)
    train.add_argument("--l2", type=float, default=0.001)
    train.set_defaults(func=cmd_train)

    rank = subparsers.add_parser("rank")
    rank.add_argument("--diagnostics", required=True)
    rank.add_argument("--model", required=True)
    rank.add_argument("--output", required=True)
    rank.add_argument("--csv-output", default="")
    rank.set_defaults(func=cmd_rank)

    demo = subparsers.add_parser("demo")
    demo.add_argument("--diagnostics", required=True)
    demo.add_argument("--output-dir", required=True)
    demo.add_argument("--metric", default="trajectory_duration_sec")
    demo.add_argument("--good-fraction", type=float, default=0.25)
    demo.add_argument("--bad-fraction", type=float, default=0.25)
    demo.add_argument("--learning-rate", type=float, default=0.08)
    demo.add_argument("--epochs", type=int, default=600)
    demo.add_argument("--l2", type=float, default=0.001)
    demo.set_defaults(func=cmd_demo)
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    raise SystemExit(args.func(args))


if __name__ == "__main__":
    main()
