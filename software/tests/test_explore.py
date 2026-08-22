"""
The properties that matter. Run before touching the drone: if these fail on a simulated room with
perfect sensors, they will fail worse on a real one with noise.

The headline is the doorway. Nothing in the code knows what a door is — no template, no label. If the
curiosity policy is right, the drone finishes the first room and crosses into the second because a
doorway is simply the biggest frontier available. If it never leaves the first room, the policy is
broken, and this catches it in seconds rather than after a crash.
"""
import math
import sim
from occupancy import OccupancyGrid
from explorer import FrontierExplorer, SAFE_M

passed = failed = 0


def ok(name, cond, detail=''):
    global passed, failed
    if cond:
        passed += 1
        print('PASS  ' + name + (' — ' + str(detail)[:110] if detail else ''))
    else:
        failed += 1
        print('FAIL  ' + name + (' — ' + str(detail)[:160] if detail else ''))


# ── the grid itself ───────────────────────────────────────────────────────
g = OccupancyGrid(4.0, 4.0, 0.10)
ok('a fresh grid is entirely unknown', g.known_fraction() == 0.0)
g.integrate_ray(0.5, 0.5, 0.0, 1.0, 3.0)
ok('one beam is NOT enough to declare space free', not g.is_free(*g.to_cell(1.0, 0.5)),
   'deliberate — a single ToF reading is evidence, not proof')
g.integrate_ray(0.5, 0.5, 0.0, 1.0, 3.0)
ok('a second confirming beam does declare it free', g.is_free(*g.to_cell(1.0, 0.5)))
ok('a return marks the surface as blocked', g.is_occupied(*g.to_cell(1.5, 0.5)))

g2 = OccupancyGrid(4.0, 4.0, 0.10)
g2.integrate_ray(0.5, 0.5, 0.0, 3.0, 3.0)
ok('a max-range reading does NOT invent a wall at sensor range',
   not g2.is_occupied(*g2.to_cell(3.4, 0.5)), 'otherwise every map gets ringed with a phantom wall')

g3 = OccupancyGrid(4.0, 4.0, 0.10)
for _ in range(12):
    g3.integrate_ray(0.5, 0.5, 0.0, 3.0, 3.0)          # seen free repeatedly
g3.integrate_ray(0.5, 0.5, 0.0, 1.0, 3.0)              # one spurious hit
ok('one bad reading cannot conjure a wall in open space',
   not g3.is_occupied(*g3.to_cell(1.0, 0.5)), 'log-odds accumulate; a single outlier only nudges')

# ── the safety rule ───────────────────────────────────────────────────────
g4 = OccupancyGrid(4.0, 4.0, 0.10)
for _ in range(6):
    g4._bump(*g4.to_cell(2.0, 2.0), 1.0)               # a known obstacle
ex4 = FrontierExplorer(g4)
ok('a cell right beside a wall is refused', not ex4.is_safe(*g4.to_cell(2.1, 2.0)))
ok('a cell beyond the standoff is allowed', ex4.is_safe(*g4.to_cell(2.6, 2.0)))
ok('unknown space is not treated as blocked',
   ex4.is_safe(*g4.to_cell(0.5, 0.5)), 'else the drone refuses to move at power-on')

# ── the whole behaviour, in a two-room flat ───────────────────────────────
r = sim.run(max_steps=2500, verbose=False)
# It stops when it stops LEARNING, not when every cell is known — frontier cells hard against a wall
# can never be cleared, so waiting for zero frontiers waits forever.
ok('it finishes on its own — no infinite wander', r['steps'] < 2500, 'stopped learning at step ' + str(r['steps']))
ok('it maps most of the flat', r['known'] > 0.55, '{:.0f}% mapped'.format(r['known'] * 100))
ok('IT FINDS THE DOORWAY ON ITS OWN and enters the second room',
   'right' in r['rooms'], 'rooms visited: ' + ', '.join(sorted(r['rooms'])))

# Every cell it ever occupied must have respected the standoff.
grid = r['grid']
ex = FrontierExplorer(grid)
ok('it ended somewhere legal', ex.is_safe(*grid.to_cell(r['pose'][0], r['pose'][1])),
   'clearance {:.2f}m, rule {:.2f}m'.format(r['min_clearance'], SAFE_M))

# SENSOR GEOMETRY. The naive assumption "more beams is faster" is NOT what the simulator shows, so this
# asserts the effect that IS reproducible: when the angular gap between rays exceeds the grid cell size,
# free space comes out speckled — each isolated speck becomes its own frontier and the drone spends its
# life chasing dots. 24 beams over 90 degrees at 3m leaves 20cm gaps between rays on 10cm cells and maps
# WORSE than the owner's 2-beam sensor. Closing the gap to one cell fixes it dramatically.
# (Single runs per configuration, and the relationship is not monotonic past that point — treat the
# sweet spot as "measure it", not as a law.)
old = (sim.N_BEAMS, sim.FOV, sim.MAX_RANGE)

sim.N_BEAMS, sim.FOV, sim.MAX_RANGE = 24, math.radians(90), 3.0     # 20cm gaps on 10cm cells
sparse = sim.run(max_steps=2500, verbose=False)

sim.N_BEAMS, sim.FOV, sim.MAX_RANGE = 48, math.radians(90), 3.0     # gaps == cell size
matched = sim.run(max_steps=2500, verbose=False)

sim.N_BEAMS, sim.FOV, sim.MAX_RANGE = old
ok('rays spaced wider than a grid cell produce a worse map',
   matched['known'] > sparse['known'],
   '20cm ray gap: {:.0f}% in {} steps  |  10cm ray gap: {:.0f}% in {} steps'.format(
       sparse['known'] * 100, sparse['steps'], matched['known'] * 100, matched['steps']))

print('\n===== EXPLORER: {}/{} passed ====='.format(passed, passed + failed))
raise SystemExit(1 if failed else 0)
