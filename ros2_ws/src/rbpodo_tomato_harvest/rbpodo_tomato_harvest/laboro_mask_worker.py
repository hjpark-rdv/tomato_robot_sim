"""Run persistent LaboroTomato Mask R-CNN inference outside the ROS process."""

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter


CLASS_NAMES = (
    "l_fully_ripened",
    "l_half_ripened",
    "l_green",
)
MASK_COLORS = (
    (235, 54, 54),
    (255, 166, 38),
    (62, 190, 80),
)


def render_laboro_masks(
    image: Image.Image,
    masks,
    labels,
    scores,
    *,
    score_threshold: float = 0.50,
) -> tuple[Image.Image, dict]:
    """Overlay accepted cherry-tomato instance masks on an RGB image."""
    threshold = float(score_threshold)
    if not 0.0 <= threshold <= 1.0:
        raise ValueError("score_threshold must be between 0 and 1")
    source = image.convert("RGB")
    mask_values = np.asarray(masks, dtype=bool)
    label_values = np.asarray(labels, dtype=np.int64).reshape(-1)
    score_values = np.asarray(scores, dtype=float).reshape(-1)
    if mask_values.size == 0:
        mask_values = np.empty((0, source.height, source.width), dtype=bool)
    if mask_values.ndim != 3 or mask_values.shape[1:] != (
        source.height,
        source.width,
    ):
        raise ValueError("instance masks do not match the source image")
    if not (
        len(mask_values) == len(label_values) == len(score_values)
    ):
        raise ValueError("mask, label and score counts do not match")

    output = source.copy()
    class_counts = {name: 0 for name in CLASS_NAMES}
    accepted_scores = []
    for mask, label, score in zip(
        mask_values,
        label_values,
        score_values,
    ):
        label = int(label)
        score = float(score)
        if score < threshold or not 0 <= label < len(CLASS_NAMES):
            continue
        mask_image = Image.fromarray((mask * 255).astype(np.uint8), mode="L")
        color = MASK_COLORS[label]
        tint = Image.new("RGB", output.size, color)
        alpha_mask = mask_image.point(lambda value: int(value * 0.48))
        output = Image.composite(
            Image.blend(output, tint, 0.72),
            output,
            alpha_mask,
        )
        expanded = mask_image.filter(ImageFilter.MaxFilter(7))
        outline = np.asarray(expanded, dtype=np.uint8) > np.asarray(
            mask_image,
            dtype=np.uint8,
        )
        outline_image = Image.fromarray(
            (outline * 255).astype(np.uint8),
            mode="L",
        )
        output.paste(color, mask=outline_image)
        class_counts[CLASS_NAMES[label]] += 1
        accepted_scores.append(score)

    draw = ImageDraw.Draw(output)
    count = sum(class_counts.values())
    summary_text = (
        f"LaboroTomato cherry masks: {count}  "
        f"score >= {threshold:.2f}"
    )
    text_box = draw.textbbox((0, 0), summary_text)
    text_width = text_box[2] - text_box[0]
    text_height = text_box[3] - text_box[1]
    draw.rounded_rectangle(
        (8, 8, 20 + text_width, 18 + text_height),
        radius=5,
        fill=(0, 0, 0, 180),
    )
    draw.text((14, 12), summary_text, fill=(255, 255, 255))
    return output, {
        "count": count,
        "class_counts": class_counts,
        "maximum_score": max(accepted_scores, default=0.0),
        "score_threshold": threshold,
    }


def _load_model(config_path: str, checkpoint_path: str, device: str):
    from mmdet.apis import init_detector

    model = init_detector(config_path, checkpoint_path, device=device)
    model.dataset_meta = {
        "classes": CLASS_NAMES,
        "palette": MASK_COLORS,
    }
    return model


def _infer(model, input_path: str, output_path: str, threshold: float) -> dict:
    from mmdet.apis import inference_detector

    image = Image.open(input_path).convert("RGB")
    # MMDetection's ndarray loader expects OpenCV/BGR channel order.
    bgr = np.asarray(image, dtype=np.uint8)[:, :, ::-1].copy()
    result = inference_detector(model, bgr)
    predictions = result.pred_instances.cpu()
    masks = (
        predictions.masks.numpy()
        if hasattr(predictions, "masks")
        else np.empty((0, image.height, image.width), dtype=bool)
    )
    output, summary = render_laboro_masks(
        image,
        masks,
        predictions.labels.numpy(),
        predictions.scores.numpy(),
        score_threshold=threshold,
    )
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    output.save(output_path, format="PNG")
    return summary


def _emit(event: str, **fields) -> None:
    print(json.dumps({"event": event, **fields}, ensure_ascii=False), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--device", default="cuda:0")
    args = parser.parse_args()
    started = time.monotonic()
    try:
        model = _load_model(args.config, args.checkpoint, args.device)
    except Exception as error:
        _emit("startup_error", message=f"{type(error).__name__}: {error}")
        raise SystemExit(2) from error
    _emit(
        "ready",
        device=args.device,
        load_duration_sec=time.monotonic() - started,
    )

    for line in sys.stdin:
        command = {}
        try:
            command = json.loads(line)
            if command.get("command") == "close":
                _emit("closed")
                return
            if command.get("command") != "infer":
                raise ValueError("unsupported command")
            request_id = int(command["request_id"])
            inference_started = time.monotonic()
            summary = _infer(
                model,
                str(command["input_path"]),
                str(command["output_path"]),
                float(command.get("score_threshold", 0.50)),
            )
            _emit(
                "result",
                request_id=request_id,
                output_path=str(command["output_path"]),
                duration_sec=time.monotonic() - inference_started,
                **summary,
            )
        except Exception as error:
            _emit(
                "inference_error",
                request_id=command.get("request_id"),
                message=f"{type(error).__name__}: {error}",
            )


if __name__ == "__main__":
    main()
