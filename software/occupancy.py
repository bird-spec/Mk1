"""
occupancy — the map the drone actually needs to not hit things.

Deliberately NOT a point cloud. For "stay 15cm off a wall" you need to know which cells are free,
which are blocked, and which you have not looked at yet. That is a grid of small integers, updated in
microseconds, and it is a completely different (much cheaper) problem than photogrammetric
reconstruction. The 3D cloud is a separate, slower, nice-to-have layer on top.

Cells hold LOG-ODDS rather than a boolean. One bad range reading should nudge a cell, not flip it —
ToF sensors produce occasional wild values off shiny or angled surfaces, and a boolean map would put a
phantom wall in the middle of a room and never remove it. Evidence accumulates; a cell seen free
fifteen times survives one spurious hit.

Pure stdlib on purpose: this has to run on a laptop, in CI, and eventually be portable to whatever the
companion board turns out to be. No numpy dependency to install on a field machine.
"""
import math

# Log-odds increments per observation. Deliberately asymmetric: a HIT is stronger evidence than a MISS,
# because a beam passing through proves free space only along a thin line, while a return proves
# something is really there.
L_FREE = -0.4
L_OCC = 0.85
L_MIN, L_MAX = -4.0, 4.0        # clamp so a cell can always be revised — nothing becomes permanent
L_OCC_THRESH = 0.5              # above this, treat as blocked
L_FREE_THRESH = -0.5            # below this, treat as safe to fly


class OccupancyGrid:
    def __init__(self, width_m=8.0, height_m=8.0, res_m=0.10):
        self.res = res_m
        self.w = int(round(width_m / res_m))
        self.h = int(round(height_m / res_m))
        # Flat list beats a list-of-lists for cache behaviour and makes serialising to the drone trivial.
        self.cells = [0.0] * (self.w * self.h)

    # ── coordinate helpers ────────────────────────────────────────────────
    def to_cell(self, x, y):
        return int(math.floor(x / self.res)), int(math.floor(y / self.res))

    def to_world(self, cx, cy):
        return (cx + 0.5) * self.res, (cy + 0.5) * self.res

    def inside(self, cx, cy):
        return 0 <= cx < self.w and 0 <= cy < self.h

    def _idx(self, cx, cy):
        return cy * self.w + cx

    def logodds(self, cx, cy):
        return self.cells[self._idx(cx, cy)] if self.inside(cx, cy) else 0.0

    def is_occupied(self, cx, cy):
        return self.logodds(cx, cy) > L_OCC_THRESH

    def is_free(self, cx, cy):
        return self.logodds(cx, cy) < L_FREE_THRESH

    def is_unknown(self, cx, cy):
        return L_FREE_THRESH <= self.logodds(cx, cy) <= L_OCC_THRESH

    def _bump(self, cx, cy, delta):
        if not self.inside(cx, cy):
            return
        i = self._idx(cx, cy)
        self.cells[i] = max(L_MIN, min(L_MAX, self.cells[i] + delta))

    # ── the only write path ───────────────────────────────────────────────
    def integrate_ray(self, x, y, theta, distance, max_range):
        """
        One beam: everything it passed THROUGH is free, and where it stopped is occupied.

        A reading at max_range means "nothing out there", not "a wall at exactly 4m" — marking that
        endpoint occupied would ring the whole map with a phantom wall at sensor range, which is the
        classic way a first occupancy grid comes out looking like a circle.
        """
        hit = distance < max_range - 1e-6
        d = min(distance, max_range)
        x1, y1 = x + math.cos(theta) * d, y + math.sin(theta) * d

        for cx, cy in _bresenham(*self.to_cell(x, y), *self.to_cell(x1, y1))[:-1]:
            self._bump(cx, cy, L_FREE)
        if hit:
            self._bump(*self.to_cell(x1, y1), L_OCC)

    def integrate_scan(self, pose, readings, max_range):
        """readings: [(bearing_rad_relative_to_heading, distance_m), ...]"""
        x, y, th = pose
        for bearing, dist in readings:
            self.integrate_ray(x, y, th + bearing, dist, max_range)

    # ── queries the flight policy needs ───────────────────────────────────
    def clearance(self, x, y, limit_m=1.0):
        """Distance to the nearest KNOWN obstacle. Unknown space is not treated as blocked —
        otherwise the drone refuses to move at startup, when everything is unknown."""
        cx, cy = self.to_cell(x, y)
        steps = int(limit_m / self.res)
        best = limit_m
        for dy in range(-steps, steps + 1):
            for dx in range(-steps, steps + 1):
                if self.is_occupied(cx + dx, cy + dy):
                    best = min(best, math.hypot(dx, dy) * self.res)
        return best

    def known_fraction(self):
        known = sum(1 for v in self.cells if v < L_FREE_THRESH or v > L_OCC_THRESH)
        return known / float(len(self.cells))

    def to_ascii(self, pose=None, targets=()):
        """A map you can actually look at. '#' wall, '.' free, ' ' unknown, 'D' drone, '*' target."""
        tset = {self.to_cell(tx, ty) for tx, ty in targets}
        dcell = self.to_cell(pose[0], pose[1]) if pose else None
        rows = []
        for cy in range(self.h - 1, -1, -1):          # y up, so the picture matches the room
            row = []
            for cx in range(self.w):
                if dcell == (cx, cy):
                    row.append('D')
                elif (cx, cy) in tset:
                    row.append('*')
                elif self.is_occupied(cx, cy):
                    row.append('#')
                elif self.is_free(cx, cy):
                    row.append('.')
                else:
                    row.append(' ')
            rows.append(''.join(row))
        return '\n'.join(rows)


def _bresenham(x0, y0, x1, y1):
    """Integer line. Every cell the beam crosses, in order, endpoint included."""
    pts = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        pts.append((x0, y0))
        if x0 == x1 and y0 == y1:
            return pts
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy
