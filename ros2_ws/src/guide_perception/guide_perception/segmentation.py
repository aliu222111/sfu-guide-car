"""Floor segmentation with SegFormer pretrained on ADE20K (spec section 7).

The model runs through ONNX Runtime (CPU, OpenVINO or CUDA execution provider),
exported once with ``scripts/export_models.py``. ADE20K class indices are the
0-based ids of ``nvidia/segformer-*-finetuned-ade-512-512``.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

# Classes the car may drive on.
DRIVABLE = {"floor": 3, "road": 6, "sidewalk": 11, "rug": 28, "path": 52, "runway": 54}
# Anything not drivable is an obstacle.

MEAN = np.array([0.485, 0.456, 0.406], dtype=np.float32) * 255.0
STD = np.array([0.229, 0.224, 0.225], dtype=np.float32) * 255.0


def drivable_ids(labels_path: Path | None = None) -> np.ndarray:
    """Drivable class ids, from the exported model's id2label when present."""
    if labels_path and Path(labels_path).exists():
        id2label = json.loads(Path(labels_path).read_text())
        names = {v.lower(): int(k) for k, v in id2label.items()}
        ids = [names[n] for n in DRIVABLE if n in names]
        if ids:
            return np.array(sorted(ids))
    return np.array(sorted(DRIVABLE.values()))


def preprocess(bgr: np.ndarray, width: int, height: int) -> np.ndarray:
    import cv2

    rgb = cv2.cvtColor(cv2.resize(bgr, (width, height), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
    x = (rgb.astype(np.float32) - MEAN) / STD
    return x.transpose(2, 0, 1)[None]


def labels_from_logits(logits: np.ndarray, out_hw: tuple[int, int]) -> np.ndarray:
    """(1, C, h, w) logits to an (H, W) label image at the crop's size."""
    import cv2

    lab = logits[0].argmax(axis=0).astype(np.uint8)
    return cv2.resize(lab, (out_hw[1], out_hw[0]), interpolation=cv2.INTER_NEAREST)


class FloorSegmenter:
    def __init__(self, model_path: str, input_size=(320, 180), providers=None, labels_path=None):
        import onnxruntime as ort

        avail = ort.get_available_providers()
        wanted = providers or ["CUDAExecutionProvider", "OpenVINOExecutionProvider", "CPUExecutionProvider"]
        self.session = ort.InferenceSession(model_path, providers=[p for p in wanted if p in avail])
        self.input_name = self.session.get_inputs()[0].name
        self.size = input_size
        self.drivable = drivable_ids(labels_path)

    def drivable_mask(self, bgr: np.ndarray) -> np.ndarray:
        x = preprocess(bgr, *self.size)
        logits = self.session.run(None, {self.input_name: x})[0]
        labels = labels_from_logits(logits, bgr.shape[:2])
        return np.isin(labels, self.drivable)
