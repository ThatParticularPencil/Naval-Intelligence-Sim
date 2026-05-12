from __future__ import annotations

import math
import sys
from typing import List, Tuple

import pygame

from simulation.engine import SimulationEngine
from ui import colors as C
from ui.terrain_renderer import TerrainRenderer
from utils.vec2 import Vec2


def _world_to_screen(p: Vec2, ox: float, oy: float, scale: float) -> Tuple[int, int]:
    return int(ox + p.x * scale), int(oy + p.y * scale)


class Dashboard:
    """
    Real-time tactical view + metrics strip.

    Map uses world coordinates scaled into the left pane; HUD on the right.
    """

    def __init__(self, engine: SimulationEngine, scale: float = 1.0) -> None:
        self.engine = engine
        self.scale = scale
        cfg = engine.world.cfg
        self.map_w = int(cfg.world_width * scale)
        self.map_h = int(cfg.world_height * scale)
        self.hud_w = 300
        self.win_w = self.map_w + self.hud_w
        self.win_h = self.map_h
        pygame.init()
        pygame.display.set_caption("Autonomous Contact Tracker — instrumentation view")
        self.screen = pygame.display.set_mode((self.win_w, self.win_h))
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Menlo", 13) if sys.platform == "darwin" else pygame.font.SysFont("Consolas", 14)
        self.font_small = pygame.font.SysFont("Menlo", 12) if sys.platform == "darwin" else pygame.font.SysFont("Consolas", 12)
        self._reset_button_rect = pygame.Rect(-100, -100, 1, 1)
        self._reset_button_hover = False

        self._terrain_renderer = TerrainRenderer(
            self.map_w,
            self.map_h,
            cfg.world_width,
            cfg.world_height,
            internal_scale=0.25,
            seed=42,
        )
        self._terrain_surface = self._terrain_renderer.render(engine.world.obstacles)

    def _rebuild_terrain(self) -> None:
        self._terrain_surface = self._terrain_renderer.render(self.engine.world.obstacles)

    def run(self, max_fps: float = 60.0) -> None:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        self.engine.reset()
                        self._rebuild_terrain()
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if self._reset_button_rect.collidepoint(event.pos):
                        self.engine.reset()
                        self._rebuild_terrain()
                elif event.type == pygame.MOUSEMOTION:
                    self._reset_button_hover = self._reset_button_rect.collidepoint(event.pos)

            self.engine.step()
            self._draw()
            pygame.display.flip()
            self.clock.tick(max_fps)
        pygame.quit()

    def _draw(self) -> None:
        self.screen.fill(C.HUD_BG)
        map_surf = pygame.Surface((self.map_w, self.map_h))
        map_surf.fill(C.BG_OCEAN)
        map_surf.blit(self._terrain_surface, (0, 0))
        self._draw_grid(map_surf)
        ox, oy = 0.0, 0.0
        sc = self.scale
        w = self.engine.world

        # Sensor footprint
        bx, by = _world_to_screen(w.vessel.position, ox, oy, sc)
        pygame.draw.circle(map_surf, C.SENSOR_RING, (bx, by), int(w.vessel.sensor_radius * sc), 1)

        # Trails (predicted under true for contrast)
        for tid, trail in self.engine.pred_trails.items():
            pts = [_world_to_screen(p, ox, oy, sc) for p in trail]
            if len(pts) >= 2:
                pygame.draw.lines(map_surf, C.TRAIL_PRED, False, pts, 1)
        for tid, trail in self.engine.true_trails.items():
            pts = [_world_to_screen(p, ox, oy, sc) for p in trail]
            if len(pts) >= 2:
                pygame.draw.lines(map_surf, C.TRAIL_TRUE, False, pts, 1)

        # True targets
        for t in w.targets:
            tx, ty = _world_to_screen(t.position, ox, oy, sc)
            pygame.draw.circle(map_surf, C.TARGET_TRUE, (tx, ty), max(2, int(t.radius * sc)))
            v = t.velocity
            if v.length() > 1e-3:
                tip = _world_to_screen(t.position + v.normalized() * 28.0, ox, oy, sc)
                pygame.draw.line(map_surf, C.TARGET_TRUE, (tx, ty), tip, 1)

        # Predicted tracks
        for tr in w.tracker.all_tracks():
            px, py = _world_to_screen(tr.estimated_position, ox, oy, sc)
            pygame.draw.rect(map_surf, C.TARGET_PRED, pygame.Rect(px - 3, py - 3, 6, 6), 1)

        # Noisy observations (this frame)
        for obs in self.engine.last_observations:
            mx, my = _world_to_screen(obs.measured_position, ox, oy, sc)
            pygame.draw.circle(map_surf, C.OBS_NOISY, (mx, my), 3, 1)

        # Vessel + heading
        self._draw_vessel(map_surf, w.vessel.position, w.vessel.heading_rad, ox, oy, sc)

        self.screen.blit(map_surf, (0, 0))
        self._draw_hud()
        pygame.draw.line(self.screen, C.GRID, (self.map_w, 0), (self.map_w, self.win_h), 1)

    def _draw_grid(self, surf: pygame.Surface) -> None:
        step = 80
        for x in range(0, self.map_w, step):
            pygame.draw.line(surf, C.GRID, (x, 0), (x, self.map_h), 1)
        for y in range(0, self.map_h, step):
            pygame.draw.line(surf, C.GRID, (0, y), (self.map_w, y), 1)

    def _draw_vessel(self, surf: pygame.Surface, pos: Vec2, heading: float, ox: float, oy: float, sc: float) -> None:
        bx, by = _world_to_screen(pos, ox, oy, sc)
        L = 14.0 * sc
        # Triangle pointing along heading
        tip = Vec2(math.cos(heading), math.sin(heading)) * L
        left = Vec2(math.cos(heading + 2.4), math.sin(heading + 2.4)) * (L * 0.55)
        right = Vec2(math.cos(heading - 2.4), math.sin(heading - 2.4)) * (L * 0.55)
        p0 = (bx + int(tip.x), by + int(tip.y))
        p1 = (bx + int(left.x), by + int(left.y))
        p2 = (bx + int(right.x), by + int(right.y))
        pygame.draw.polygon(surf, C.VESSEL, (p0, p1, p2))
        pygame.draw.line(surf, C.VESSEL_HEADING, (bx, by), p0, 2)

    def _draw_reset_button(self, x: int, y: int) -> int:
        """Draw Reset control; returns next y below the button."""
        w_btn, h_btn = 118, 28
        self._reset_button_rect = pygame.Rect(x, y, w_btn, h_btn)
        bg = C.BUTTON_BG_HOVER if self._reset_button_hover else C.BUTTON_BG
        pygame.draw.rect(self.screen, bg, self._reset_button_rect, border_radius=4)
        pygame.draw.rect(self.screen, C.BUTTON_BORDER, self._reset_button_rect, 1, border_radius=4)
        label = self.font.render("Reset", True, C.BUTTON_TEXT)
        lx = x + (w_btn - label.get_width()) // 2
        ly = y + (h_btn - label.get_height()) // 2
        self.screen.blit(label, (lx, ly))
        return y + h_btn + 10

    def _draw_hud(self) -> None:
        x0 = self.map_w + 10
        y = 12
        w = self.engine.world
        m = self.engine.metrics

        self.screen.blit(self.font.render("AUTONOMOUS CONTACT TRACKER", True, C.HUD_TEXT), (x0, y))
        y += 16
        y = self._draw_reset_button(x0, y)

        lines: List[str] = [
            f"t_sim: {w.sim_time:8.1f} s",
            f"tracks: {len(w.tracker.tracks)} / targets: {len(w.targets)}",
            "",
            "legend: ● true  □ pred  ○ obs",
            "",
            "--- metrics ---",
            f"mean pos err: {m.mean_position_error():6.2f} m",
            f"mean vel err: {m.mean_velocity_error():6.2f} m/s",
            f"mean reacq gap: {m.mean_reacquisition_s():5.2f} s",
            f"avg maintained: {m.mean_maintained_fraction()*100:5.1f}%",
            "",
            "--- vessel ---",
            f"speed: {w.vessel.velocity.length():5.2f} m/s",
            f"sensor R: {w.vessel.sensor_radius:.0f} m",
            "",
            "--- tracks ---",
        ]
        for line in lines:
            col = C.HUD_TEXT
            if line.startswith("---"):
                col = C.HUD_MUTED
            self.screen.blit(self.font.render(line, True, col), (x0, y))
            y += 16

        # Per-contact table
        tracks = sorted(w.tracker.all_tracks(), key=lambda t: t.contact_id)
        for tr in tracks:
            age = tr.seconds_since_observation(w.sim_time)
            row = f"{tr.contact_id}  c={tr.confidence:.2f}  Δt={age:.2f}s"
            self.screen.blit(self.font_small.render(row, True, C.HUD_TEXT), (x0, y))
            y += 15
            if y > self.win_h - 40:
                self.screen.blit(self.font_small.render("…", True, C.HUD_MUTED), (x0, y))
                break

        hint = self.font_small.render("[R] or Reset   [ESC] quit", True, C.HUD_MUTED)
        self.screen.blit(hint, (x0, self.win_h - 22))
