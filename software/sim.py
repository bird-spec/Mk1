"""
sim — fly the explorer through a fake flat before any hardware exists.

The room is deliberately TWO rooms joined by a narrow door. That is the interesting case: the drone
starts in one room, and nothing in the code knows what a door is. If the curiosity policy works, it
finishes the first room and then goes through the gap on its own, because a doorway is simply the
largest frontier available. If it never leaves the first room, the policy is broken — which is exactly
what this is here to catch.

Ground truth is raycast against real walls, so the map is built from simulated sensor readings rather
than copied from the answer.
"""
import math
from occupancy import OccupancyGrid
from explorer import FrontierExplorer

W_M, H_M, RES = 6.0, 5.0, 0.10
# The owner has 2x VL53L0X: SINGLE-POINT sensors, ~25 degree cone, ~2m reliable indoors. That is two
# beams, not a depth image — so the sim models exactly that by default. Set BEAMS/FOV/RANGE higher to
# see what a multi-zone sensor (VL53L5CX, 8x8) would buy.
MAX_RANGE = 2.0
FOV = math.radians(50)     # two sensors mounted ~50 degrees apart
N_BEAMS = 2


def build_room():
    """Walls as segments. Two rooms, one 0.7m door in the dividing wall."""
    segs = [
        ((0.2, 0.2), (5.8, 0.2)),      # south
        ((0.2, 4.8), (5.8, 4.8)),      # north
        ((0.2, 0.2), (0.2, 4.8)),      # west
        ((5.8, 0.2), (5.8, 4.8)),      # east
        ((3.0, 0.2), (3.0, 2.0)),      # divider, lower half
        ((3.0, 2.7), (3.0, 4.8)),      # divider, upper half -> a 0.7m doorway at y≈2.0-2.7
        ((1.0, 3.4), (2.0, 3.4)),      # a table in the first room
    ]
    return segs


def raycast(segs, x, y, theta, max_range):
    """Distance to the nearest wall along a heading; max_range means 'nothing out there'."""
    best = max_range
    dx, dy = math.cos(theta), math.sin(theta)
    for (ax, ay), (bx, by) in segs:
        sx, sy = bx - ax, by - ay
        den = dx * sy - dy * sx
        if abs(den) < 1e-9:
            continue
        t = ((ax - x) * sy - (ay - y) * sx) / den      # along the ray
        u = ((ax - x) * dy - (ay - y) * dx) / den      # along the segment
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best


def blocked(segs, x0, y0, x1, y1, margin=0.06):
    """Would this move cross a REAL wall?

    The sim was only collision-checking against the MAP, so the drone flew straight through any wall it
    had not discovered yet and 'explored' places it could never have reached — one run ended outside
    the building. A real drone hits the wall whether or not it knew about it, and a simulator that is
    more forgiving than reality is worse than no simulator.
    """
    d = math.hypot(x1 - x0, y1 - y0)
    if d < 1e-9:
        return False
    th = math.atan2(y1 - y0, x1 - x0)
    return raycast(segs, x0, y0, th, d + margin) < d + margin


def scan(segs, pose):
    x, y, th = pose
    out = []
    for i in range(N_BEAMS):
        bearing = -FOV / 2 + FOV * i / (N_BEAMS - 1)
        out.append((bearing, raycast(segs, x, y, th + bearing, MAX_RANGE)))
    return out


def run(max_steps=600, verbose=True):
    """
    Stop, look around, then go somewhere interesting — repeat.

    This shape is forced by the hardware. With a 2-beam sensor each stationary reading reveals two thin
    lines; the only way to see a room is to ROTATE. An earlier version only swept when it got stuck and
    re-planned every single step, which made the drone flip-flop between two targets 0.4m apart and map
    4% of the room in 400 steps. Committing to a target and sweeping on arrival is both more realistic
    and what actually works.
    """
    segs = build_room()
    grid = OccupancyGrid(W_M, H_M, RES)
    ex = FrontierExplorer(grid)

    pose = (1.0, 1.0, 0.0)          # start in the LEFT room, facing east
    visited_rooms, steps = set(), 0

    def sweep(p, turns=12):
        """A full turn on the spot, integrating as it goes. This is the mapping, not the travel."""
        nonlocal steps
        for _ in range(turns):
            p = ex.sweep_heading(p, 2 * math.pi / turns)
            grid.integrate_scan(p, scan(segs, p), MAX_RANGE)
            steps += 1
        return p

    pose = sweep(pose)
    # DIMINISHING RETURNS. There are always frontier cells hard against a wall whose unknown neighbour
    # is INSIDE the wall — they can never be cleared, because standing close enough to see them breaks
    # the standoff. Waiting for zero frontiers means waiting forever, grinding on 20-cell scraps. Real
    # exploration stops when it stops learning, and 100% coverage of anything is a fantasy.
    stale, last_known = 0, grid.known_fraction()
    while steps < max_steps:
        visited_rooms.add('right' if pose[0] > 3.0 else 'left')
        now_known = grid.known_fraction()
        if now_known - last_known < 0.004:          # under 0.4% gained since the last target
            stale += 1
            if stale >= 6:
                if verbose:
                    print('\nexploration complete — %d targets with no real gain' % stale)
                break
        else:
            stale = 0
        last_known = now_known
        t = ex.next_target(pose)
        if not t:
            if verbose:
                print(f"\nnothing left worth looking at after {steps} steps")
            break
        if verbose:
            print(f"[{steps:3d}] curious about ({t[0]:.1f}, {t[1]:.1f}) — {t[2]}")

        # COMMIT to the target. Re-planning every step is what caused the oscillation; a real drone
        # that changes its mind ten times a second never gets anywhere either.
        target, guard = (t[0], t[1]), 0
        while math.hypot(target[0] - pose[0], target[1] - pose[1]) > 0.3 and guard < 40 and steps < max_steps:
            nxt = ex.step_toward(pose, target)
            if not nxt:
                break                                   # blocked en route; sweep and re-choose
            if blocked(segs, pose[0], pose[1], nxt[0], nxt[1]):
                # LEARN FROM THE BUMP. Something is there whether or not the beams found it, so write it
                # into the map. Without this the planner re-picks the same unreachable frontier forever:
                # target, blocked, sweep, same target — 2500 steps of trying to walk through a wall.
                # On the real drone this is the ToF suddenly reading very close, or a bump switch.
                for _ in range(4):
                    grid._bump(*grid.to_cell(nxt[0], nxt[1]), 1.0)
                break
            pose = nxt
            grid.integrate_scan(pose, scan(segs, pose), MAX_RANGE)
            steps += 1
            guard += 1
        pose = sweep(pose)                              # arrived (or gave up): look around

    # Never finish a run parked inside the standoff — see FrontierExplorer.retreat_to_safe.
    _before = pose
    pose = ex.retreat_to_safe(pose)
    if blocked(segs, _before[0], _before[1], pose[0], pose[1]):
        pose = _before
    return {
        'steps': steps,
        'known': grid.known_fraction(),
        'rooms': visited_rooms,
        'grid': grid,
        'pose': pose,
        'min_clearance': min(grid.clearance(pose[0], pose[1]), 1.0),
    }


if __name__ == '__main__':
    r = run()
    print("\n" + r['grid'].to_ascii(r['pose']))
    print(f"\nsteps={r['steps']}  mapped={r['known']*100:.0f}%  rooms visited={sorted(r['rooms'])}")
