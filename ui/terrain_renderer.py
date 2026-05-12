"""
Procedural rocky terrain / island shading from circular obstacles.

Generates at reduced resolution, then upscales. All heavy work is NumPy
broadcasting over a low-res grid (no per-pixel Python loops).
"""

from __future__ import annotations

from typing import Any, Iterable, Sequence, Tuple

import numpy as np
import pygame

# --- small vectorized primitives -------------------------------------------------


def _smoothstep(edge0: np.ndarray | float, edge1: np.ndarray | float, x: np.ndarray) -> np.ndarray:
    """Element-wise smoothstep; safe when edge0 < edge1."""
    denom = np.maximum(edge1 - edge0, 1e-9)
    t = np.clip((x - edge0) / denom, 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def _hash01(ix: np.ndarray, iy: np.ndarray, seed: int) -> np.ndarray:
    """Deterministic [0, 1) hash for integer lattice coords (same shape as ix)."""
    xf = ix.astype(np.float64)
    yf = iy.astype(np.float64)
    return np.mod(np.sin(xf * 12.9898 + yf * 78.233 + float(seed)) * 43758.5453, 1.0)


def _value_noise_2d(x: np.ndarray, y: np.ndarray, seed: int) -> np.ndarray:
    """Smooth interpolated value noise on a unit grid; x,y arbitrary float arrays."""
    xi = np.floor(x).astype(np.int64)
    yi = np.floor(y).astype(np.int64)
    xf = x - xi
    yf = y - yi
    u = xf * xf * (3.0 - 2.0 * xf)
    v = yf * yf * (3.0 - 2.0 * yf)

    a = _hash01(xi, yi, seed)
    b = _hash01(xi + 1, yi, seed)
    c = _hash01(xi, yi + 1, seed)
    d = _hash01(xi + 1, yi + 1, seed)

    x1 = a + (b - a) * u
    x2 = c + (d - c) * u
    return x1 + (x2 - x1) * v


def _fbm_2d(x: np.ndarray, y: np.ndarray, seed: int, octaves: int) -> np.ndarray:
    """Fractional Brownian motion from value noise, ~[0, 1]."""
    amp = 1.0
    freq = 1.0
    tot = np.zeros_like(x, dtype=np.float64)
    wsum = 0.0
    for o in range(octaves):
        tot += amp * _value_noise_2d(x * freq, y * freq, seed + o * 101)
        wsum += amp
        freq *= 2.0
        amp *= 0.5
    return tot / max(wsum, 1e-9)


def _ridged_multifractal(x: np.ndarray, y: np.ndarray, seed: int, octaves: int = 5) -> np.ndarray:
    """
    Ridged multifractal: each octave uses (1 - |n|) with n ~ [-1, 1] from scaled noise.
    Produces sharp ridges / craggy rock appearance.
    """
    out = np.zeros_like(x, dtype=np.float64)
    wsum = np.zeros_like(x, dtype=np.float64)
    f = 1.0
    a = 1.0
    for o in range(octaves):
        n = _value_noise_2d(x * f, y * f, seed + o * 131) * 2.0 - 1.0
        ridge = 1.0 - np.abs(n)
        out += ridge * a
        wsum += a
        f *= 2.05
        a *= 0.52
    return out / np.maximum(wsum, 1e-9)


def _obstacle_circles(obstacles: Iterable[Any]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Extract (cx, cy, r) float arrays from duck-typed obstacles."""
    cx_list: list[float] = []
    cy_list: list[float] = []
    r_list: list[float] = []
    for o in obstacles:
        rad: float | None = None
        if hasattr(o, "radius"):
            rad = float(o.radius)
        elif hasattr(o, "rect"):
            rect = o.rect
            rad = 0.5 * float(max(rect.width, rect.height))
        else:
            rad = 16.0

        if hasattr(o, "center"):
            c = o.center
            if hasattr(c, "x") and hasattr(c, "y"):
                cx_list.append(float(c.x))
                cy_list.append(float(c.y))
            else:
                cx_list.append(float(c[0]))
                cy_list.append(float(c[1]))
        elif hasattr(o, "pos"):
            p = o.pos
            if hasattr(p, "x") and hasattr(p, "y"):
                cx_list.append(float(p.x))
                cy_list.append(float(p.y))
            else:
                cx_list.append(float(p[0]))
                cy_list.append(float(p[1]))
        elif hasattr(o, "rect"):
            rect = o.rect
            cx_list.append(float(rect.centerx))
            cy_list.append(float(rect.centery))
        else:
            continue

        r_list.append(rad)

    if not cx_list:
        return (
            np.zeros(0, dtype=np.float64),
            np.zeros(0, dtype=np.float64),
            np.zeros(0, dtype=np.float64),
        )
    return (
        np.asarray(cx_list, dtype=np.float64),
        np.asarray(cy_list, dtype=np.float64),
        np.asarray(r_list, dtype=np.float64),
    )


def _nearest_center_distance_and_radius(
    X: np.ndarray, Y: np.ndarray, cx: np.ndarray, cy: np.ndarray, radii: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """
    For each cell, Euclidean distance to nearest obstacle center and that obstacle's radius.

    Vectorized: dists has shape (N, H, W), take argmin along axis 0.
    """
    if cx.size == 0:
        return (
            np.full(X.shape, np.inf, dtype=np.float64),
            np.ones(X.shape, dtype=np.float64),
        )
    # (N, 1, 1) for broadcasting with (H, W)
    cx_b = cx[:, np.newaxis, np.newaxis]
    cy_b = cy[:, np.newaxis, np.newaxis]
    dx = X[np.newaxis, ...] - cx_b
    dy = Y[np.newaxis, ...] - cy_b
    dists = np.hypot(dx, dy)
    idx = np.argmin(dists, axis=0)
    d_min = dists.min(axis=0)
    r_near = radii[idx]
    return d_min.astype(np.float64), r_near.astype(np.float64)


def _height_to_rgba(h: np.ndarray, mask: np.ndarray, water_alpha_thresh: float = 0.04) -> np.ndarray:
    """
    Map normalized height h in [0,1] and island mask to RGBA uint8 (H, W, 4).

    Water: transparent. Slopes: dark grey. Peaks: light grey / white.
    """
    h = np.clip(h, 0.0, 1.0)
    m = np.clip(mask, 0.0, 1.0)

    c_deep = np.array([38, 42, 48], dtype=np.float64)
    c_mid = np.array([72, 78, 86], dtype=np.float64)
    c_peak = np.array([210, 212, 218], dtype=np.float64)

    t = np.clip((h - 0.35) / 0.45, 0.0, 1.0)
    rock_mid_peak = (1.0 - t)[..., np.newaxis] * c_mid + t[..., np.newaxis] * c_peak
    t2 = np.clip((0.45 - h) / 0.35, 0.0, 1.0)
    rock_rgb = (1.0 - t2)[..., np.newaxis] * rock_mid_peak + t2[..., np.newaxis] * c_deep

    edge = np.clip((m - water_alpha_thresh) / 0.08, 0.0, 1.0)
    alpha = np.where(m < water_alpha_thresh, 0, np.clip(m * 255.0, 0, 255)).astype(np.float64)
    alpha = np.maximum(alpha, edge * 255.0)

    rgb = np.clip(rock_rgb, 0.0, 255.0).astype(np.uint8)
    rgba = np.zeros((*h.shape, 4), dtype=np.uint8)
    rgba[:, :, 0:3] = rgb
    rgba[:, :, 3] = np.clip(alpha, 0, 255).astype(np.uint8)
    return np.ascontiguousarray(rgba)


class TerrainRenderer:
    """
    Build a single pygame.Surface of procedural rocky islands from circular obstacles.

    Usage::

        renderer = TerrainRenderer(map_w, map_h, world_w, world_h, internal_scale=0.25)
        terrain_surface = renderer.render(world.obstacles)

    Blit ``terrain_surface`` first; ocean / grid draw beneath or around as desired.
    """

    def __init__(
        self,
        output_width: int,
        output_height: int,
        world_width: float,
        world_height: float,
        *,
        internal_scale: float = 0.25,
        seed: int = 0,
        warp_strength: float = 14.0,
        warp_freq: float = 0.011,
        ridge_octaves: int = 6,
        water_alpha_thresh: float = 0.04,
    ) -> None:
        self.out_w = max(1, int(output_width))
        self.out_h = max(1, int(output_height))
        self.world_w = float(world_width)
        self.world_h = float(world_height)
        self.internal_scale = float(np.clip(internal_scale, 0.05, 1.0))
        self.seed = int(seed)
        self.warp_strength = float(warp_strength)
        self.warp_freq = float(warp_freq)
        self.ridge_octaves = int(ridge_octaves)
        self.water_alpha_thresh = float(water_alpha_thresh)

        self._lw = max(2, int(round(self.out_w * self.internal_scale)))
        self._lh = max(2, int(round(self.out_h * self.internal_scale)))

    def render(self, obstacles: Sequence[Any]) -> pygame.Surface:
        """
        Generate full-resolution RGBA terrain. Safe to call when obstacle layout changes.

        Returns a surface with SRCALPHA; water pixels have alpha 0.
        """
        cx, cy, radii = _obstacle_circles(obstacles)
        lw, lh = self._lw, self._lh

        # Cell centers in world space (vectorized grid)
        jj, ii = np.indices((lh, lw), dtype=np.float64)
        X = (ii + 0.5) / lw * self.world_w
        Y = (jj + 0.5) / lh * self.world_h

        # Domain warp (small FBM offsets in world meters)
        s = self.seed
        wfx = X * self.warp_freq
        wfy = Y * self.warp_freq
        off_x = (_fbm_2d(wfx, wfy, s + 3, 3) - 0.5) * 2.0
        off_y = (_fbm_2d(wfx + 19.2, wfy + 11.7, s + 7, 3) - 0.5) * 2.0
        Xw = X + self.warp_strength * off_x
        Yw = Y + self.warp_strength * off_y

        d_min, r_near = _nearest_center_distance_and_radius(Xw, Yw, cx, cy, radii)

        # Island mask from distance to nearest *center* (per user spec)
        lo = r_near * 0.8
        hi = r_near * 1.2
        mask = 1.0 - _smoothstep(lo, hi, d_min)

        if cx.size == 0:
            mask = np.zeros_like(d_min, dtype=np.float64)

        # Ridged rock texture in a stable world-frequency space
        tex_scale = 0.018
        ridge = _ridged_multifractal(Xw * tex_scale, Yw * tex_scale, s + 900, self.ridge_octaves)

        # Height composite: mask modulates ridged structure + subtle base
        base = _fbm_2d(Xw * 0.006 + 1.7, Yw * 0.006 - 0.4, s + 40, 4)
        h_raw = mask * (0.22 + 0.78 * ridge) + 0.12 * mask * base
        # Emphasize cliffs at mask gradient (optional, vectorized Sobel-like via diff)
        gx = np.abs(np.diff(h_raw, axis=1, prepend=h_raw[:, :1]))
        gy = np.abs(np.diff(h_raw, axis=0, prepend=h_raw[:1, :]))
        cliff = np.clip((gx + gy) * 3.5, 0.0, 1.0)
        h = np.clip(h_raw * (0.82 + 0.35 * cliff), 0.0, 1.0)

        rgba = _height_to_rgba(h, mask, self.water_alpha_thresh)

        low_surf = pygame.image.frombuffer(rgba.tobytes(), (lw, lh), "RGBA")
        # Own pixel data (buffer would be invalid after return otherwise)
        low_surf = low_surf.copy()

        if lw == self.out_w and lh == self.out_h:
            return low_surf

        return pygame.transform.smoothscale(low_surf, (self.out_w, self.out_h))
