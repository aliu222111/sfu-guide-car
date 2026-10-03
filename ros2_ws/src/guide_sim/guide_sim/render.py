"""Camera images of the simulated building, as the car's webcams would see them.

One ray per image column finds the wall (2.5D ray casting: walls are vertical
and full height), then every pixel is floor, wall, person or ceiling from its
row. Surfaces carry texture that is a hash of world position, so it never
repeats (repeating texture makes loop closure find false matches) and the two
cameras see the same features. The level image is warped to the camera's
real downward tilt with a rotation homography, which is exact for a pinhole
camera.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import cv2
import numpy as np

from .world import FloorWorld

CEILING = 2.7  # m
PERSON_HEIGHT = 1.7


@dataclass
class CameraModel:
    width: int = 640
    height: int = 360
    fx: float = 460.0  # about 70 deg across: a typical 720p webcam captured at 640 x 360
    fy: float = 460.0
    cx: float = 320.0
    cy: float = 180.0
    # Mount on base_link (match the URDF: cam_x, cam_z, cam_pitch, stereo_baseline).
    x: float = 0.30
    z: float = 0.45
    pitch: float = 0.20  # rad, nose down
    baseline: float = 0.12
    noise: float = 3.0  # grey levels of sensor noise

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1.0]])


def _hash(i: np.ndarray, j: np.ndarray, k: int) -> np.ndarray:
    """Uniform [0, 1) per integer lattice point (a 32-bit integer mix, no repeats)."""
    h = (i.astype(np.uint64) * np.uint64(0x9E3779B1) + j.astype(np.uint64) * np.uint64(0x85EBCA77)
         + np.uint64(k * 0xC2B2AE3D + 0x27D4EB2F)) & np.uint64(0xFFFFFFFF)
    h ^= h >> np.uint64(15)
    h = (h * np.uint64(0x2C1B3C6D)) & np.uint64(0xFFFFFFFF)
    h ^= h >> np.uint64(12)
    h = (h * np.uint64(0x297A2D39)) & np.uint64(0xFFFFFFFF)
    h ^= h >> np.uint64(15)
    return (h & np.uint64(0xFFFF)).astype(np.float32) / 65536.0


def smooth_texture(a: np.ndarray, b: np.ndarray, kind: int) -> np.ndarray:
    """Value noise interpolated between lattice points: speckle and blotches, no straight edges.

    For floor and ceiling. Blocky texture there lays a grid of straight lines
    along the world axes, which the vanishing-point detector takes for the
    corridor (real floors are untiled, or tiled along the corridor).
    """
    out = np.zeros(a.shape, np.float32)
    for scale, weight in ((0.5, 0.4), (0.12, 0.35), (0.03, 0.25)):
        u, v = a / scale, b / scale
        i, j = np.floor(u), np.floor(v)
        fu, fv = (u - i).astype(np.float32), (v - j).astype(np.float32)
        fu, fv = fu * fu * (3 - 2 * fu), fv * fv * (3 - 2 * fv)
        k = kind * 7 + int(scale * 100)
        h00, h10 = _hash(i, j, k), _hash(i + 1, j, k)
        h01, h11 = _hash(i, j + 1, k), _hash(i + 1, j + 1, k)
        out += weight * ((h00 * (1 - fu) + h10 * fu) * (1 - fv) + (h01 * (1 - fu) + h11 * fu) * fv)
    return out


def texture(a: np.ndarray, b: np.ndarray, kind: int) -> np.ndarray:
    """Blocky multi-scale value noise at world coordinates (a, b): plenty of corners."""
    out = np.zeros(a.shape, np.float32)
    for scale, weight in ((0.6, 0.45), (0.15, 0.35), (0.04, 0.2)):
        out += weight * _hash(np.floor(a / scale), np.floor(b / scale), kind * 7 + int(scale * 100))
    return out


class Renderer:
    def __init__(self, cam: CameraModel | None = None, seed: int = 0):
        self.cam = cam or CameraModel()
        c = self.cam
        # Level image tall and wide enough to cover the tilted view (tilting also
        # moves the image edges sideways).
        self.lh = int(c.height * 2.2)
        self.lcy = self.lh / 2
        margin = int(0.15 * c.width)
        self.lw = c.width + 2 * margin
        self.lcx = c.cx + margin
        self.cols = np.arange(self.lw) + 0.5
        self.col_angle = np.arctan2(-(self.cols - self.lcx), c.fx)  # left of the axis is positive
        self.col_norm = np.hypot(1.0, (self.cols - self.lcx) / c.fx)  # horizontal ray length per unit forward depth
        rows = np.arange(self.lh) + 0.5
        self.row_slope = (rows - self.lcy) / c.fy  # down is positive, per unit forward depth
        # Homography from the level image to the tilted one: x_t = K R K_l^-1 x_l.
        Kl = np.array([[c.fx, 0, self.lcx], [0, c.fy, self.lcy], [0, 0, 1.0]])
        cp, sp = math.cos(c.pitch), math.sin(c.pitch)
        # Optical frame (x right, y down, z forward). Tilted nose down, a level
        # forward ray appears above the image centre: y_t = cp*y - sp*z.
        R = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
        self.H = c.K @ R @ np.linalg.inv(Kl)
        self.rng = np.random.default_rng(seed)

    def camera_pose(self, x: float, y: float, yaw: float, side: str) -> tuple[float, float]:
        c = self.cam
        lat = {"left": c.baseline / 2, "right": -c.baseline / 2}.get(side, 0.0)  # "center": the iPhone
        cs, sn = math.cos(yaw), math.sin(yaw)
        return x + cs * c.x - sn * lat, y + sn * c.x + cs * lat

    def render(self, world: FloorWorld, x: float, y: float, yaw: float, side: str, t: float = 0.0,
               with_depth: bool = False):
        """The grey image; with ``with_depth`` also each pixel's depth (m, along the optical axis)."""
        c = self.cam
        px, py = self.camera_pose(x, y, yaw, side)
        hit = world.raycast(px, py, yaw + self.col_angle, max_range=25.0, t=t)
        # Forward depth of the wall in each column; past max range draw a far wall
        # rather than floor to the horizon.
        depth = np.where(np.isfinite(hit.distance), hit.distance, 25.0) / self.col_norm
        slope = self.row_slope[:, None]  # (rows, 1)
        # Height of the ray at the wall: z = cam_z - depth * slope.
        z_wall = c.z - depth[None, :] * slope
        is_person = (hit.person >= 0)[None, :]
        height_limit = np.where(is_person, PERSON_HEIGHT, CEILING)
        img = np.empty((self.lh, self.lw), np.float32)
        wall = (z_wall >= 0) & (z_wall <= height_limit)
        floor = z_wall < 0
        ceiling = ~wall & ~floor
        # Floor and ceiling: where the pixel's ray meets z = 0 or z = CEILING.
        dirx = np.cos(yaw + self.col_angle) * self.col_norm
        diry = np.sin(yaw + self.col_angle) * self.col_norm
        with np.errstate(divide="ignore", invalid="ignore"):
            t_floor = c.z / slope
            t_ceil = (c.z - CEILING) / slope
        fx = px + t_floor * dirx[None, :]
        fy = py + t_floor * diry[None, :]
        img[floor] = 0.15 + 0.6 * smooth_texture(fx[floor], fy[floor], 1)
        cxw = px + t_ceil * dirx[None, :]
        cyw = py + t_ceil * diry[None, :]
        cm = ceiling
        img[cm] = 0.65 + 0.25 * smooth_texture(cxw[cm], cyw[cm], 2)
        u = np.broadcast_to(hit.u[None, :], z_wall.shape)
        face = np.broadcast_to(hit.face[None, :], z_wall.shape)
        tex = texture(u[wall], z_wall[wall], 3)
        shade = np.where(face[wall] == 4, 0.15 + 0.3 * tex, 0.45 + 0.4 * tex)
        # Skirting board: a dark band at the bottom of walls, as in most corridors.
        shade = np.where((face[wall] < 4) & (z_wall[wall] < 0.1), 0.2, shade)
        # Light fall-off with distance so near and far walls differ.
        dist = np.broadcast_to(depth[None, :], z_wall.shape)[wall]
        img[wall] = shade * (1.0 - 0.25 * np.clip(dist / 20.0, 0, 1))
        level = np.clip(img * 255.0, 0, 255).astype(np.uint8)
        out = cv2.warpPerspective(level, self.H, (c.width, c.height), flags=cv2.INTER_LINEAR)
        if c.noise > 0:
            out = np.clip(out.astype(np.int16) + self.rng.normal(0, c.noise, out.shape).astype(np.int16), 0, 255).astype(np.uint8)
        if not with_depth:
            return out
        # Forward distance along the level axis, then along the tilted optical axis:
        # a point (X, slope*z, z) in the level frame has z_t = sin(p)*slope*z + cos(p)*z.
        z_level = np.where(wall, np.broadcast_to(depth[None, :], z_wall.shape), np.where(floor, t_floor, t_ceil))
        z_tilt = (z_level * (math.sin(c.pitch) * slope + math.cos(c.pitch))).astype(np.float32)
        return out, cv2.warpPerspective(z_tilt, self.H, (c.width, c.height), flags=cv2.INTER_NEAREST)
