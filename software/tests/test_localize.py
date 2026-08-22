"""
The claim under test: once the room is mapped, a 2m sensor is enough to know where you are in a room
bigger than 2m — because the MAP fills in what the sensor cannot reach.

The hard version is the "kidnapped robot" case. Build the map, then drop the drone somewhere random
with NO idea where it is, and see whether four short-range beams can find it. If that works, the
everyday case (drifting slowly from a known start) is trivial by comparison.
"""
import math
import random

import sim
from localize import ParticleFilter

passed = failed = 0


def ok(name, cond, detail=''):
    global passed, failed
    if cond:
        passed += 1
        print('PASS  ' + name + (' — ' + str(detail)[:120] if detail else ''))
    else:
        failed += 1
        print('FAIL  ' + name + (' — ' + str(detail)[:170] if detail else ''))


# ── build the map first, exactly as the drone would ───────────────────────
print('mapping the flat...')
run = sim.run(max_steps=2500, verbose=False)
grid = run['grid']
segs = sim.build_room()
ok('we have a map to localize against', run['known'] > 0.55, '{:.0f}% mapped'.format(run['known'] * 100))

# Four short-range beams — the owner's actual 4-pack, pointed front/left/right/back.
SENSOR_RANGE = 2.0
BEARINGS = [0.0, math.pi / 2, -math.pi / 2, math.pi]


def read(pose):
    """What the real sensors would report from this true pose, with noise."""
    rng = random.Random(int(pose[0] * 1000) + int(pose[1] * 1000))
    out = []
    for b in BEARINGS:
        d = sim.raycast(segs, pose[0], pose[1], pose[2] + b, SENSOR_RANGE)
        out.append((b, max(0.02, d + rng.gauss(0, 0.02))))
    return out


def truth_error(est, truth):
    return math.hypot(est['x'] - truth[0], est['y'] - truth[1])


# ── the out-of-range claim, checked directly ──────────────────────────────
pf0 = ParticleFilter(grid, n=300, seed=1)
pf0.scatter_global()
mid = (2.0, 2.5, 0.0)
r_mid = read(mid)
oor = [d for _, d in r_mid if d >= SENSOR_RANGE - 0.05]
ok('from the middle of the flat, some walls ARE out of sensor reach',
   len(oor) > 0, '{} of {} beams saw nothing within {}m'.format(len(oor), len(BEARINGS), SENSOR_RANGE))

# ── the kidnapped-robot test ──────────────────────────────────────────────
random.seed(7)
truth = (1.6, 1.2, 0.4)                  # dropped here; the filter is told nothing
pf = ParticleFilter(grid, n=800, seed=7)
pf.scatter_global()
est = pf.estimate()
ok('with no prior it is honestly uncertain', not est['confident'],
   'spread {:.2f}m across the whole flat'.format(est['spread']))

# Drive a short path, feeding it only what the real sensors would see.
path = [(0.0, 0.5), (0.25, 0.0), (0.0, -0.9), (0.3, 0.0), (0.0, 0.6), (0.25, 0.0),
        (0.0, 0.7), (0.3, 0.0), (0.0, -0.5), (0.25, 0.0), (0.0, 0.4), (0.3, 0.0)]
for d_th, d_fwd in path:
    th = truth[2] + d_th
    truth = (truth[0] + math.cos(th) * d_fwd, truth[1] + math.sin(th) * d_fwd, th)
    est = pf.update(d_fwd, d_th, read(truth), SENSOR_RANGE)

err = truth_error(est, truth)
ok('IT FINDS ITSELF from a cold start with only 2m beams', err < 0.6,
   'off by {:.2f}m after {} moves (truth {:.1f},{:.1f})'.format(err, len(path), truth[0], truth[1]))
ok('and it knows that it knows', est['confident'], 'spread {:.2f}m'.format(est['spread']))

# ── the everyday case: known start, gyro drifting ─────────────────────────
truth2 = (1.0, 1.0, 0.0)
pf2 = ParticleFilter(grid, n=500, seed=3)
pf2.scatter_near(truth2)
dead = list(truth2)                       # what pure dead-reckoning would believe
drift = 0.06                              # rad of yaw error per move — an MPU6050 with no compass
est2 = None
for i in range(16):
    d_th, d_fwd = (0.35, 0.28) if i % 3 else (0.0, 0.3)
    th = truth2[2] + d_th
    truth2 = (truth2[0] + math.cos(th) * d_fwd, truth2[1] + math.sin(th) * d_fwd, th)
    # Dead reckoning believes the COMMAND, and quietly accumulates the yaw error.
    dead[2] += d_th + drift
    dead[0] += math.cos(dead[2]) * d_fwd
    dead[1] += math.sin(dead[2]) * d_fwd
    est2 = pf2.update(d_fwd, d_th, read(truth2), SENSOR_RANGE)

dead_err = math.hypot(dead[0] - truth2[0], dead[1] - truth2[1])
pf_err = truth_error(est2, truth2)
ok('dead reckoning drifts away, as it must', dead_err > 0.3, 'gyro-only is off by {:.2f}m'.format(dead_err))
ok('the filter stays locked on despite the same drift', pf_err < dead_err,
   'filter {:.2f}m vs dead reckoning {:.2f}m'.format(pf_err, dead_err))

print('\n===== LOCALIZATION: {}/{} passed ====='.format(passed, passed + failed))
raise SystemExit(1 if failed else 0)
