# CAD

Not yet designed.

Planned: a `.STEP` assembly of the airframe with sensor and board mounts, so the
sensor mounting positions are reproducible rather than cardboard-and-guesswork.

The four ToF beams have to sit at exactly 90 degrees to each other, level, and
out past the prop discs - `software/localize.py` assumes the beams are where the
code says they are, so mount tolerance is a correctness problem and not a
cosmetic one.
