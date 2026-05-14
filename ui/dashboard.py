from __future__ import annotations

import math
import sys

import pygame

from simulation.engine import SimulationEngine
from utils.config import SimulationConfig as cfg
from ui import colors as C
from utils.vec2 import Vec2


def _world_to_screen(p: Vec2, ox: float, oy: float, scale: float) -> tuple[int, int]:
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
        self.font_score = pygame.font.SysFont("Menlo", 30, bold=True) if sys.platform == "darwin" else pygame.font.SysFont("Consolas", 30, bold=True)
        self._reset_button_rect = pygame.Rect(-100, -100, 1, 1)
        self._reset_button_hover = False
        self.wave_history: list[tuple[Vec2, float]] = []  # (position, creation_time)

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
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if self._reset_button_rect.collidepoint(event.pos):
                        self.engine.reset()
                elif event.type == pygame.MOUSEMOTION:
                    self._reset_button_hover = self._reset_button_rect.collidepoint(event.pos)

            self.engine.step()
            # Add new waves from all vessels and targets
            sim_time = self.engine.world.sim_time
            for vessel in self.engine.world.vessels:
                offset = Vec2(vessel.radius * 1.2, 0).rotate_rad(vessel.heading)
                self.wave_history.append(((vessel.position + offset).copy(), sim_time))
            for t in self.engine.world.targets:
                offset = Vec2(t.radius*1.2,0).rotate_rad(t.heading)
                self.wave_history.append(((t.position + offset).copy(), sim_time))
            # Remove old waves (older than 3 seconds)
            self.wave_history = [(pos, t) for pos, t in self.wave_history if sim_time - t < 3.0]
            self._draw()
            pygame.display.flip()
            self.clock.tick(max_fps)
        pygame.quit()

    def _draw(self) -> None:
        self.screen.fill(C.HUD_BG)
        map_surf = pygame.Surface((self.map_w, self.map_h))
        map_surf.fill(C.BG_OCEAN)
        self._draw_grid(map_surf)

        ox, oy = 0.0, 0.0
        sc = self.scale
        w = self.engine.world
        cfg = w.cfg  # use instance, not the imported class
        
        # Wave trails from targets
        self._draw_all_waves(map_surf, w.sim_time, ox, oy, sc)

        # Obstacles
        for ob in w.obstacles:
            cx, cy = _world_to_screen(ob.center, ox, oy, sc)
            pygame.draw.circle(map_surf, C.OBSTACLE, (cx, cy), int(ob.radius * sc))

        # Sensor footprints
        for vessel in w.vessels:
            bx, by = _world_to_screen(vessel.position, ox, oy, sc)
            pygame.draw.circle(map_surf, C.SENSOR_RING, (bx, by), int(cfg.vessel_sensor_radius * sc), 1)

        # Trails — pred drawn first (underneath)
        for trail in self.engine.pred_trails.values():
            pts = [_world_to_screen(p, ox, oy, sc) for p, _ in trail]
            if len(pts) >= 2:
                pygame.draw.lines(map_surf, C.TRAIL_PRED, False, pts, 1)

        # for trail in self.engine.true_trails.values():
        #     pts = [_world_to_screen(p, ox, oy, sc) for p, _ in trail]
        #     if len(pts) >= 2:
        #         pygame.draw.lines(map_surf, C.TRAIL_TRUE, False, pts, 1)


        # True targets
        for t in w.targets:
            self._draw_target_triangle(
                map_surf, t.position, t.velocity, t.heading,
                ox, oy, sc, C.TARGET_TRUE,
                size=max(8.0, t.radius * 2.0),
                outline=False,
            )

        # Global predicted tracks
        for tr in w.tracker.all_tracks():
            px, py = _world_to_screen(tr.estimated_position, ox, oy, sc)
            pygame.draw.rect(map_surf, C.TARGET_PRED, pygame.Rect(px - 4, py - 4, 8, 8), 1)
            pygame.draw.line(map_surf, C.TARGET_PRED, (px - 6, py), (px + 6, py), 1)
            pygame.draw.line(map_surf, C.TARGET_PRED, (px, py - 6), (px, py + 6), 1)

        # Noisy observations
        for obs in self.engine.last_observations:
            mx, my = _world_to_screen(obs.measured_position, ox, oy, sc)
            pygame.draw.circle(map_surf, C.OBS_NOISY, (mx, my), 3, 1)

        # All vessels + heading
        for vessel in w.vessels:
            self._draw_vessel(map_surf, vessel.position, vessel.heading, vessel.radius, ox, oy, sc)

        self.screen.blit(map_surf, (0, 0))
        self._draw_hud()
        pygame.draw.line(self.screen, C.GRID, (self.map_w, 0), (self.map_w, self.win_h), 1)

    def _draw_grid(self, surf: pygame.Surface) -> None:
        step = 30
        for x in range(0, self.map_w, step):
            pygame.draw.line(surf, C.GRID, (x, 0), (x, self.map_h), 1)
        for y in range(0, self.map_h, step):
            pygame.draw.line(surf, C.GRID, (0, y), (self.map_w, y), 1)

    def _draw_vessel(self, surf: pygame.Surface, pos: Vec2, heading: float, radius: float, ox: float, oy: float, sc: float) -> None:
        bx, by = _world_to_screen(pos, ox, oy, sc)
        L = max(8.0, radius * 3.0) * sc
        # Triangle pointing along heading
        tip = Vec2(math.cos(heading), math.sin(heading)) * L
        left = Vec2(math.cos(heading + 2.4), math.sin(heading + 2.4)) * (L * 0.55)
        right = Vec2(math.cos(heading - 2.4), math.sin(heading - 2.4)) * (L * 0.55)
        p0 = (bx + int(tip.x), by + int(tip.y))
        p1 = (bx + int(left.x), by + int(left.y))
        p2 = (bx + int(right.x), by + int(right.y))
        pygame.draw.polygon(surf, C.VESSEL, (p0, p1, p2))
        pygame.draw.line(surf, C.VESSEL_HEADING, (bx, by), p0, 2)

    def _draw_target_triangle(
        self,
        surf: pygame.Surface,
        pos: Vec2,
        vel: Vec2,
        heading: float,
        ox: float,
        oy: float,
        sc: float,
        color: tuple[int, int, int],
        size: float = 10.0,
        outline: bool = False,
    ) -> None:
        bx, by = _world_to_screen(pos, ox, oy, sc)
        L = size * sc
        # heading = math.atan2(vel.y, vel.x) if vel.length() > 1e-3 else -math.pi / 2
        tip = Vec2(math.cos(heading), math.sin(heading)) * L
        left = Vec2(math.cos(heading + 2.4), math.sin(heading + 2.4)) * (L * 0.55)
        right = Vec2(math.cos(heading - 2.4), math.sin(heading - 2.4)) * (L * 0.55)
        p0 = (bx + int(tip.x), by + int(tip.y))
        p1 = (bx + int(left.x), by + int(left.y))
        p2 = (bx + int(right.x), by + int(right.y))
        if outline:
            pygame.draw.polygon(surf, color, (p0, p1, p2), 1)
        else:
            pygame.draw.polygon(surf, color, (p0, p1, p2))

    def _draw_all_waves(self, surf: pygame.Surface, sim_time: float, ox: float, oy: float, sc: float) -> None:
        max_age = 2.5
        max_radius = 15.0
        if sim_time <= .1:
            self.wave_history.clear()

        for pos, creation_time in self.wave_history:
            age = sim_time - creation_time
            if age < 0 or age >= max_age:
                continue
            alpha_factor = max(0.0, 1.0 - age / max_age)**3

            if alpha_factor <= 0:
                continue

            bx, by = _world_to_screen(pos, ox, oy, sc)
            radius = int(math.sqrt(age / max_age) * max_radius * sc) + 2

            wave_surf = pygame.Surface((radius * 2, radius * 2), pygame.SRCALPHA)
            alpha = int(alpha_factor * 255)
            pygame.draw.circle(wave_surf, (*C.WAVE, alpha), (radius, radius), radius, 1)
            surf.blit(wave_surf, (bx - radius, by - radius))

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

        vessel_speeds = [v.velocity.length() for v in w.vessels]
        avg_vessel_speed = sum(vessel_speeds) / max(1, len(vessel_speeds))

        lines: list[str] = [
            f"t_sim: {w.sim_time:8.1f} s",
            f"tracks: {len(w.tracker.tracks)} / targets: {len(w.targets)}",
            "",
            "legend: △ true  ⊞ pred  ○ obs",
            "",
            "--- metrics ---",
            f"mean pos err: {m.mean_position_error():6.2f} m",
            f"mean vel err: {m.mean_velocity_error():6.2f} m/s",
            f"mean reacq gap: {m.mean_reacquisition_s():5.2f} s",
            f"avg maintained: {m.mean_maintained_fraction()*100:5.1f}%",
            "",
            "--- vessels ---",
            f"count: {len(w.vessels)}",
            f"avg speed: {avg_vessel_speed:5.2f} m/s",
            f"sensor R: {cfg.vessel_sensor_radius:.0f} m",
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
            chaser = self.engine.active_chase_assignments.get(tr.contact_id, "--")
            row = f"{tr.contact_id}  c={tr.confidence:.2f}  Δt={age:.2f}s  {chaser}"
            self.screen.blit(self.font_small.render(row, True, C.HUD_TEXT), (x0, y))
            y += 15
            if y > self.win_h - 120:
                self.screen.blit(self.font_small.render("…", True, C.HUD_MUTED), (x0, y))
                break

        self._draw_score_panel(x0)

        hint = self.font_small.render("[R] or Reset   [ESC] quit", True, C.HUD_MUTED)
        self.screen.blit(hint, (x0, self.win_h - 22))

    def _draw_score_panel(self, x: int) -> None:
        w = self.engine.world
        tracked_ids = {
            track.contact_id
            for track in w.tracker.all_tracks()
            if track.confidence >= w.cfg.track_maintained_conf_threshold
        }
        all_targets_tracked = bool(w.targets) and w.true_target_ids().issubset(tracked_ids)

        panel = pygame.Rect(x, self.win_h - 104, self.hud_w - 20, 70)
        pulse = (math.sin(w.sim_time * 7.0) + 1.0) * 0.5 if all_targets_tracked else 0.0
        border = C.SCORE_GLOW if all_targets_tracked and pulse > 0.35 else C.SCORE
        width = 2 + int(pulse * 3)

        if all_targets_tracked:
            glow = pygame.Surface((panel.width + 12, panel.height + 12), pygame.SRCALPHA)
            alpha = 45 + int(pulse * 70)
            pygame.draw.rect(glow, (*C.SCORE_GLOW, alpha), glow.get_rect(), border_radius=8)
            self.screen.blit(glow, (panel.x - 6, panel.y - 6))

        pygame.draw.rect(self.screen, C.HUD_BG, panel, border_radius=6)
        pygame.draw.rect(self.screen, border, panel, width, border_radius=6)

        label = self.font_small.render("TRACKING SCORE", True, C.HUD_MUTED)
        score = self.font_score.render(f"{self.engine.tracking_score:07.1f}", True, C.SCORE)
        self.screen.blit(label, (panel.x + 10, panel.y + 8))
        self.screen.blit(score, (panel.x + 10, panel.y + 26))

        if all_targets_tracked:
            bonus = self.font_small.render("ALL TARGETS TRACKED", True, C.SCORE_GLOW)
            self.screen.blit(bonus, (panel.right - bonus.get_width() - 10, panel.y + 10))
