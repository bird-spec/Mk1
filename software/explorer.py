"""
explorer — the "curious" half. Decides where to look next, and refuses to fly somewhere that would
break the 15cm rule.

Two ideas:

  FRONTIERS. The boundary between "I know this is free" and "I have not looked there" is exactly where
  new information lives. Fly to a frontier and you are guaranteed to learn something; fly anywhere else
  and you mostly re-observe what you already know. This is why the drone heads for a doorway on its own
  without anyone teaching it what a doorway is — a door is just the biggest frontier in the room.

  CURIOSITY = INFORMATION PER METRE. Nearest-frontier is the usual textbook choice and it produces a
  fidgety drone that clears trivial specks next to itself. Scoring each candidate by how much unknown
  space it would actually reveal, DIVIDED by how far it must fly to get there, produces something that
  reads as intent: it will cross a room to peer through a door rather than tidy up the corner it is in.

Safety is deliberately NOT in this file's decisions alone. The 15cm rule is enforced here when planning,
but it must ALSO be enforced on the drone against a live rangefinder. Anything that depends on a WiFi
round-trip is a suggestion, not a safety system.
"""
import math
from collections import deque

SAFE_M = 0.15          # the 15cm standoff — a hard constraint on any planned waypoint
MIN_TRAVEL_M = 0.35    # ignore frontiers you are practically on top of; turn to see those instead
GAIN_RADIUS_M = 0.6    # how far around a frontier we credit as "would become visible"


class FrontierExplorer:
    def __init__(self, grid, safe_m=SAFE_M, plan_m=0.10):
        self.grid = grid
        self.safe_m = safe_m     # live reflex: how close the drone may ever BE to a wall
        self.plan_m = plan_m     # planning: how narrow a gap it may route THROUGH

    # ── where is there something to learn ─────────────────────────────────
    def frontier_cells(self):
        """A frontier cell is FREE (so reachable and safe to be near) and touches UNKNOWN."""
        g = self.grid
        out = []
        for cy in range(g.h):
            for cx in range(g.w):
                if not g.is_free(cx, cy):
                    continue
                for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                    if g.inside(cx + dx, cy + dy) and g.is_unknown(cx + dx, cy + dy):
                        out.append((cx, cy))
                        break
        return out

    def cluster(self, cells, min_size=3):
        """
        Group touching frontier cells. Without this every cell is its own candidate and the drone
        dithers between 200 near-identical targets one cell apart. min_size drops single-cell specks,
        which are usually sensor noise rather than real openings.
        """
        todo, clusters = set(cells), []
        while todo:
            seed = todo.pop()
            group, q = [seed], deque([seed])
            while q:
                cx, cy = q.popleft()
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        n = (cx + dx, cy + dy)
                        if n in todo:
                            todo.discard(n)
                            group.append(n)
                            q.append(n)
            if len(group) >= min_size:
                clusters.append(group)
        return clusters

    # ── how interesting is it, and can we get there ───────────────────────
    def information_gain(self, cx, cy):
        """Unknown cells within sensor reach of this spot — what standing here would reveal."""
        g = self.grid
        r = int(GAIN_RADIUS_M / g.res)
        n = 0
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r and g.is_unknown(cx + dx, cy + dy):
                    n += 1
        return n

    def is_safe(self, cx, cy):
        """Would sitting here violate the standoff? Checked against KNOWN obstacles only —
        treating unknown as blocked freezes the drone at power-on, when all of it is unknown."""
        g = self.grid
        r = int(math.ceil(self.safe_m / g.res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue
                if g.is_occupied(cx + dx, cy + dy):
                    return False
        return True

    def can_pass(self, cx, cy):
        """Planning clearance — roughly the drone's own half-width, NOT the comfort standoff.

        Using the 15cm standoff for routing inflates every wall by 15cm on both sides. A 70cm doorway
        becomes 40cm of 'safe' cells, and once ray scatter smears the mapped walls a cell wider, it
        closes completely. The planner then reports 'nowhere left to go' while sitting in an open flat,
        and the drone parks in the room it took off from looking perfectly content. Worse, it only
        happens once mapping SUCCEEDS — getting better at mapping is what breaks it.
        """
        g = self.grid
        r = int(math.ceil(self.plan_m / g.res))
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r and g.is_occupied(cx + dx, cy + dy):
                    return False
        return True

    def _reachable(self, start_cell):
        """BFS over free-and-safe cells. Returns {cell: steps}. Also answers 'can I even get there',
        which a straight-line distance cannot — a frontier through a wall is worthless."""
        g = self.grid
        dist = {start_cell: 0}
        q = deque([start_cell])
        while q:
            cur = q.popleft()
            cx, cy = cur
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                n = (cx + dx, cy + dy)
                if n in dist or not g.inside(*n):
                    continue
                # Unknown cells are traversable in planning — otherwise the drone can never enter the
                # very space it is trying to explore. Safety is re-checked at the waypoint itself.
                if g.is_occupied(*n) or not self.can_pass(*n):
                    continue
                dist[n] = dist[cur] + 1
                q.append(n)
        return dist

    def next_target(self, pose):
        """
        The curiosity decision: highest information-per-metre among REACHABLE frontier clusters.
        Returns (x, y, why) or None when there is nothing left worth seeing.
        """
        g = self.grid
        start = g.to_cell(pose[0], pose[1])
        clusters = self.cluster(self.frontier_cells())
        if not clusters:
            return None
        reach = self._reachable(start)

        best, best_score, best_meta = None, 0.0, None
        for group in clusters:
            # Judge the cluster by its most informative reachable member.
            # PERF: score ONE representative per cluster, not every cell in it.
            #
            # information_gain() scans ~169 cells, and this ran it for EVERY frontier cell in EVERY
            # cluster. Early on there are ~40 frontier cells and it is fine; as the map fragments there
            # are 500+, so each decision costs ~100k operations and the run gets slower the longer it
            # goes — works, then crawls, then looks hung. Clusters are contiguous by construction so
            # their members score near-identically; the member nearest the centre loses nothing.
            #
            # MIN_TRAVEL_M still applies: a frontier you are standing ON is unknown because it is
            # BEHIND the sensor, and the fix is to turn, not fly to it. Scoring those made the drone
            # oscillate between two cells 10cm apart forever.
            usable = [c for c in group if c in reach and reach[c] * g.res >= MIN_TRAVEL_M]
            if usable:
                # Sample evenly across the cluster rather than taking one central cell. A frontier that
                # wraps a corner has its centroid in a wall, and judging the whole cluster by that cell
                # threw away the members worth flying to (61% mapped instead of 88%).
                step = max(1, len(usable) // 5)
                for cell in usable[::step]:
                    cost = reach[cell] * g.res
                    gain = self.information_gain(*cell) + len(group)   # openness counts too
                    score = gain / (cost + 0.5)                        # +0.5 so adjacent aren't infinite
                    if score > best_score:
                        best_score, best = score, cell
                        best_meta = (gain, cost, len(group))
        if not best:
            return None
        x, y = g.to_world(*best)
        gain, cost, size = best_meta
        return x, y, f"{gain} unknown cells within reach, {cost:.1f}m away, opening {size} cells wide"

    def step_toward(self, pose, target, step_m=0.25):
        """
        One safe waypoint toward the target. Prefers the direct heading, but will take the best
        sideways option rather than fly into the standoff zone — so a wall makes it slide along
        instead of stalling nose-first against it.
        """
        g = self.grid
        x, y, _ = pose
        tx, ty = target[0], target[1]
        want = math.atan2(ty - y, tx - x)
        spreads = (0.0, 0.4, -0.4, 0.8, -0.8, 1.2, -1.2, 1.6, -1.6)
        # TWO PASSES. First insist on the full 15cm standoff; only if every heading is blocked do we
        # accept the tighter planning clearance — that is the doorway case, where squeezing through is
        # the whole point. Using can_pass for BOTH passes silently dropped the standoff everywhere and
        # had the drone cruising 10cm off walls in open space.
        for check in (self.is_safe, self.can_pass):
            for spread in spreads:
                th = want + spread
                nx, ny = x + math.cos(th) * step_m, y + math.sin(th) * step_m
                c = g.to_cell(nx, ny)
                if g.inside(*c) and not g.is_occupied(*c) and check(*c):
                    return nx, ny, th
        return None      # boxed in — the caller should hold position, not guess


    def retreat_to_safe(self, pose, max_steps=12, step_m=0.15):
        """Back off until the full standoff is satisfied.

        Squeezing through a doorway at less than 15cm is fine as a TRANSIT — you are through in a
        moment. Coming to rest there is not: the drone ends a run hovering 14cm off a wall, on a rule
        that promises 15cm, and any drift from that position is a strike. So whenever it stops — end of
        exploration, lost target, low battery — it walks back out to legal clearance first.
        """
        g = self.grid
        x, y, th = pose
        for _ in range(max_steps):
            if self.is_safe(*g.to_cell(x, y)):
                return (x, y, th)
            # Head directly away from whatever is nearest.
            best, bd = None, 1e9
            cx, cy = g.to_cell(x, y)
            r = int(math.ceil(self.safe_m / g.res)) + 1
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    if g.is_occupied(cx + dx, cy + dy):
                        d = dx * dx + dy * dy
                        if d < bd:
                            bd, best = d, (dx, dy)
            if not best:
                return (x, y, th)
            away = math.atan2(-best[1], -best[0])
            nx, ny = x + math.cos(away) * step_m, y + math.sin(away) * step_m
            if not g.inside(*g.to_cell(nx, ny)) or g.is_occupied(*g.to_cell(nx, ny)):
                return (x, y, th)
            x, y = nx, ny
        return (x, y, th)

    def sweep_heading(self, pose, turn_rad=0.52):
        """Rotate in place to bring the sensor's field of view onto new ground.

        With a narrow forward sensor (a couple of VL53L0X beams, say) most of the world is behind you at
        any moment. A real curious drone stops and looks around; so does this. Returns a new pose with
        the same position and a turned heading.
        """
        return (pose[0], pose[1], pose[2] + turn_rad)
