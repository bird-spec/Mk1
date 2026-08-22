"""
localize — where am I, given a map I already built?

This is the answer to "the sensor can't reach the far wall, but the map already knows what's over
there". Once a map exists, you no longer need to SEE a landmark to use it. You need enough readings to
be consistent with only one place in the map.

Monte Carlo Localization: hold a cloud of guesses about your pose. For each guess, predict what the
sensors WOULD read from there by raycasting the map, and compare against what they actually read.
Guesses that disagree lose weight and get resampled away. What survives is where you are.

Two properties that matter for a 2m sensor in a 5m room:

  OUT-OF-RANGE READINGS STILL CONSTRAIN. "Nothing within 2m" is not a missing measurement — it is
  positive evidence that no wall is near, and it kills every guess that predicted one. Weak sensors are
  not useless sensors once you have a map to check them against.

  IT RECOVERS FROM BEING LOST. Gyro yaw drift accumulates forever and there is no compass on an
  MPU6050, so dead reckoning WILL eventually be wrong. A particle filter does not care how it got lost:
  scatter guesses again and the readings sort it out. That is the property dead reckoning can never
  have.

What it cannot do is tell you about things that were not there when you mapped — a moved chair, a
person walking past, a closed door. The map is a prior, never a substitute for looking.
"""
import math
import random

from occupancy import OccupancyGrid


class ParticleFilter:
    def __init__(self, grid, n=600, seed=None):
        self.grid = grid
        self.n = n
        self.rng = random.Random(seed)
        self.particles = []          # [(x, y, theta, weight), ...]

    # ── starting out ──────────────────────────────────────────────────────
    def scatter_global(self):
        """No idea where we are: spread guesses over every free cell. This is also the recovery move
        when tracking is lost, which is why it is worth having even if you normally start from a
        known launch point."""
        g = self.grid
        free = [(cx, cy) for cy in range(g.h) for cx in range(g.w) if g.is_free(cx, cy)]
        if not free:
            return False
        self.particles = []
        for _ in range(self.n):
            cx, cy = self.rng.choice(free)
            x, y = g.to_world(cx, cy)
            self.particles.append([
                x + self.rng.uniform(-g.res / 2, g.res / 2),
                y + self.rng.uniform(-g.res / 2, g.res / 2),
                self.rng.uniform(-math.pi, math.pi),
                1.0 / self.n,
            ])
        return True

    def scatter_near(self, pose, spread_m=0.3, spread_rad=0.3):
        """We think we know roughly where we are — e.g. sitting on the launch pad."""
        self.particles = [[
            pose[0] + self.rng.gauss(0, spread_m),
            pose[1] + self.rng.gauss(0, spread_m),
            pose[2] + self.rng.gauss(0, spread_rad),
            1.0 / self.n,
        ] for _ in range(self.n)]

    # ── the two updates ───────────────────────────────────────────────────
    def motion_update(self, d_forward, d_theta, noise_m=0.04, noise_rad=0.06):
        """
        Apply the movement we BELIEVE we made, with noise, because we never move exactly as commanded.
        The noise is the point: it is what stops every particle collapsing onto one wrong answer and
        lets the cloud keep covering the truth.
        """
        for p in self.particles:
            th = p[2] + d_theta + self.rng.gauss(0, noise_rad)
            step = d_forward + self.rng.gauss(0, noise_m)
            p[0] += math.cos(th) * step
            p[1] += math.sin(th) * step
            p[2] = th

    def _expected_range(self, x, y, theta, max_range):
        """What would a beam from here read, according to the MAP? March until it hits a known
        obstacle; if it never does, the honest answer is max_range."""
        g = self.grid
        step = g.res * 0.5
        d = 0.0
        while d < max_range:
            d += step
            cx, cy = g.to_cell(x + math.cos(theta) * d, y + math.sin(theta) * d)
            if not g.inside(cx, cy):
                return d
            if g.is_occupied(cx, cy):
                return d
        return max_range

    def sensor_update(self, readings, max_range, sigma=0.25):
        """
        readings: [(bearing_rad, distance_m), ...] exactly as the flight code sees them.

        A guess that predicts the world you are actually observing keeps its weight; one that does not,
        loses it. Out-of-range is compared like any other value — see the module docstring.
        """
        if not self.particles:
            return
        total = 0.0
        for p in self.particles:
            logw = 0.0
            for bearing, actual in readings:
                expect = self._expected_range(p[0], p[1], p[2] + bearing, max_range)
                err = actual - expect
                logw += -(err * err) / (2 * sigma * sigma)
            # A small floor stops one bad beam from annihilating an otherwise good guess — real sensors
            # produce outliers, and a filter with no tolerance for them diverges on the first glitch.
            p[3] = math.exp(logw) + 1e-12
            total += p[3]
        if total > 0:
            for p in self.particles:
                p[3] /= total

    def resample(self):
        """Low-variance resampling: keep the guesses that explain the readings, drop the rest.

        The systematic (comb) method rather than n independent draws — it introduces far less sampling
        noise, which matters when the particle count is small enough to run on a companion board.
        """
        if not self.particles:
            return
        n = len(self.particles)
        step = 1.0 / n
        r = self.rng.uniform(0, step)
        c = self.particles[0][3]
        i = 0
        out = []
        for m in range(n):
            u = r + m * step
            while u > c and i < n - 1:
                i += 1
                c += self.particles[i][3]
            src = self.particles[i]
            out.append([src[0], src[1], src[2], 1.0 / n])
        self.particles = out

    # ── the answer ────────────────────────────────────────────────────────
    def estimate(self):
        """Weighted mean pose, plus how spread out the cloud is.

        The spread is the useful half: it is the filter telling you how much to trust itself. Tight
        cloud, believe it. Wide cloud, you are lost and should say so rather than fly on a guess.
        """
        if not self.particles:
            return None
        wsum = sum(p[3] for p in self.particles) or 1.0
        x = sum(p[0] * p[3] for p in self.particles) / wsum
        y = sum(p[1] * p[3] for p in self.particles) / wsum
        # Headings are circular — averaging 359 and 1 degree must give 0, not 180.
        sx = sum(math.cos(p[2]) * p[3] for p in self.particles) / wsum
        sy = sum(math.sin(p[2]) * p[3] for p in self.particles) / wsum
        th = math.atan2(sy, sx)
        spread = math.sqrt(sum(((p[0] - x) ** 2 + (p[1] - y) ** 2) * p[3] for p in self.particles) / wsum)
        return {'x': x, 'y': y, 'theta': th, 'spread': spread, 'confident': spread < 0.35}

    def update(self, d_forward, d_theta, readings, max_range):
        """One full cycle. Call it every time the drone moves and reads."""
        self.motion_update(d_forward, d_theta)
        self.sensor_update(readings, max_range)
        self.resample()
        return self.estimate()
