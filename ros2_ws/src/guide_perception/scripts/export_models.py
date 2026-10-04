#!/usr/bin/env python3
"""Download the pretrained models once and export them for the car (spec sections 7, 13, 15).

    pip install torch transformers ultralytics onnx
    python export_models.py --out models --segformer b0     # b2 on an NVIDIA laptop
    python export_models.py --out models --only yolo        # people only: pip install ultralytics onnx
    python export_models.py --out models --only school      # YOLOE with campus objects (bins, carts, signs...)

Run from the repository root. Writes models/segformer-<size>-ade.onnx (+ .labels.json),
models/yolo11n.onnx and keeps the downloaded models/yolo11n.pt (for an NVIDIA GPU).
Nothing is trained.
"""

import argparse
import json
from pathlib import Path


def export_segformer(size: str, out: Path, width: int, height: int):
    import torch
    from transformers import SegformerForSemanticSegmentation

    name = f"nvidia/segformer-{size}-finetuned-ade-512-512"
    model = SegformerForSemanticSegmentation.from_pretrained(name).eval()

    class Logits(torch.nn.Module):
        def __init__(self, m):
            super().__init__()
            self.m = m

        def forward(self, x):
            return self.m(pixel_values=x).logits

    path = out / f"segformer-{size}-ade.onnx"
    torch.onnx.export(
        Logits(model),
        torch.zeros(1, 3, height, width),
        str(path),
        input_names=["pixel_values"],
        output_names=["logits"],
        dynamic_axes={"pixel_values": {2: "h", 3: "w"}, "logits": {2: "h4", 3: "w4"}},
        opset_version=17,
    )
    (out / f"segformer-{size}-ade.labels.json").write_text(json.dumps(model.config.id2label, indent=0))
    print("wrote", path)


def export_yolo(out: Path):
    from ultralytics import YOLO

    # Ultralytics downloads the weights into the path given: keep them in models/, not the repo root
    path = YOLO(str(out / "yolo11n.pt")).export(format="onnx", imgsz=640)
    target = out / Path(path).name
    Path(path).replace(target)
    print("wrote", target)


def export_school(out: Path):
    """YOLOE with the campus object names built in: an ONNX the car runs like any YOLO model."""
    import sys

    from ultralytics import YOLOE

    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
    from guide_perception.people import SCHOOL_CLASSES

    m = YOLOE(str(out / "yoloe-11s-seg.pt"))
    m.set_classes(SCHOOL_CLASSES, m.get_text_pe(SCHOOL_CLASSES))
    path = Path(m.export(format="onnx", imgsz=640))
    target = out / "yoloe-school.onnx"
    path.replace(target)
    print("wrote", target, "with", len(SCHOOL_CLASSES), "classes")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="models")
    ap.add_argument("--segformer", default="b0", choices=["b0", "b1", "b2"])
    ap.add_argument("--width", type=int, default=320)
    ap.add_argument("--height", type=int, default=180)
    ap.add_argument("--only", choices=["yolo", "segformer", "school"], help="export just one model")
    ap.add_argument("--skip-yolo", action="store_true", help="same as --only segformer")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    if a.only == "school":
        export_school(out)
    else:
        if a.only != "yolo":
            export_segformer(a.segformer, out, a.width, a.height)
        if a.only != "segformer" and not a.skip_yolo:
            export_yolo(out)
