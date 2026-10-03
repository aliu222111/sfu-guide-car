"""Align an RTAB-Map floor database with the GIS map frame (spec section 6).

Drive to at least three known door nodes, note RTAB-Map's position at each,
and solve the 2D similarity (scale, rotation, translation) taking RTAB-Map
coordinates to GIS map coordinates. Stored per floor in ``alignments.yaml``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml


@dataclass
class Similarity2D:
    scale: float
    yaw: float
    tx: float
    ty: float
    rms: float = 0.0

    def apply(self, x, y):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.scale * (c * x - s * y) + self.tx, self.scale * (s * x + c * y) + self.ty

    def apply_yaw(self, yaw):
        return yaw + self.yaw

    def inverse(self) -> "Similarity2D":
        c, s = math.cos(-self.yaw), math.sin(-self.yaw)
        k = 1.0 / self.scale
        tx = -k * (c * self.tx - s * self.ty)
        ty = -k * (s * self.tx + c * self.ty)
        return Similarity2D(k, -self.yaw, tx, ty, self.rms)

    def as_dict(self):
        return {"scale": float(self.scale), "yaw": float(self.yaw), "tx": float(self.tx), "ty": float(self.ty), "rms": float(self.rms)}


def solve(src, dst, allow_scale: bool = True) -> Similarity2D:
    """Least-squares similarity taking ``src`` points onto ``dst`` (Umeyama 1991)."""
    a = np.asarray(src, dtype=float)
    b = np.asarray(dst, dtype=float)
    if a.shape != b.shape or a.shape[0] < 2 or a.shape[1] != 2:
        raise ValueError("need at least two matching 2D point pairs")
    ma, mb = a.mean(0), b.mean(0)
    A, B = a - ma, b - mb
    cov = B.T @ A / len(a)
    U, S, Vt = np.linalg.svd(cov)
    D = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        D[1, 1] = -1
    R = U @ D @ Vt
    var_a = (A**2).sum() / len(a)
    scale = float(np.trace(np.diag(S) @ D) / var_a) if allow_scale else 1.0
    t = mb - scale * R @ ma
    yaw = math.atan2(R[1, 0], R[0, 0])
    sim = Similarity2D(scale, yaw, float(t[0]), float(t[1]))
    pred = np.array([sim.apply(*p) for p in a])
    sim.rms = float(np.sqrt(((pred - b) ** 2).sum(1).mean()))
    return sim


def load_alignments(path: Path | str) -> dict[str, Similarity2D]:
    p = Path(path)
    if not p.exists():
        return {}
    raw = yaml.safe_load(p.read_text()) or {}
    return {floor: Similarity2D(**{k: v[k] for k in ("scale", "yaw", "tx", "ty", "rms") if k in v}) for floor, v in raw.items()}


def save_alignment(path: Path | str, floor: str, sim: Similarity2D, pairs=None):
    p = Path(path)
    raw = yaml.safe_load(p.read_text()) if p.exists() else {}
    raw = raw or {}
    entry = sim.as_dict()
    if pairs:
        entry["pairs"] = [{"node": n, "rtabmap": [float(u), float(v)]} for n, (u, v) in pairs]
    raw[floor] = entry
    p.write_text(yaml.safe_dump(raw, sort_keys=True))


def main(argv=None):
    """``align_floor --floor AQ-03 --graph maps/route_graph.json AQ-03:door:3150=12.1,3.4 ...``"""
    import argparse
    import json

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--floor", required=True)
    ap.add_argument("--graph", required=True, help="maps/route_graph.json")
    ap.add_argument("--out", default="alignments.yaml")
    ap.add_argument("--no-scale", action="store_true", help="fix scale at 1 (stereo is metric)")
    ap.add_argument("pairs", nargs="+", help="NODE=x,y with x,y RTAB-Map coordinates at that node")
    args = ap.parse_args(argv)
    with open(args.graph) as f:
        nodes = {n["id"]: n for n in json.load(f)["nodes"]}
    src, dst, pairs = [], [], []
    for item in args.pairs:
        nid, xy = item.split("=")
        u, v = (float(c) for c in xy.split(","))
        n = nodes[nid]
        src.append((u, v))
        dst.append((n["x"], n["y"]))
        pairs.append((nid, (u, v)))
    if len(src) < 3:
        ap.error("spec section 6 asks for three known doors")
    sim = solve(src, dst, allow_scale=not args.no_scale)
    save_alignment(args.out, args.floor, sim, pairs)
    print(f"{args.floor}: scale {sim.scale:.4f}, yaw {math.degrees(sim.yaw):.2f} deg, t ({sim.tx:.2f}, {sim.ty:.2f}), rms {sim.rms:.2f} m")
    if sim.rms > 1.0:
        print("warning: rms above 1 m, check the door positions")
