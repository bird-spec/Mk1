# PCB

Not yet designed. The current build is hand-soldered jumper wire.

Planned: a single carrier board the XIAO plugs into, carrying

  - both DRV8833 drivers, with IN2/IN4 tied to ground at the board
  - four ToF headers with XSHUT broken out to separate MCU pins
  - the MPU6050
  - battery pads feeding motors directly, never through the XIAO

`docs/WIRING.md` is effectively the netlist for this board - every connection
the schematic needs is already specified there.
