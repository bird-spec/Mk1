# Nevlar scout drone — wiring reference

XIAO ESP32S3 Sense.
**D5 is dead** (pad shorted internally) — never use it.
Serial runs over native USB, so **D6 and D7 are free GPIO**.

**No servos.** The camera is fixed, facing forward. The aircraft yaws to look
around, which is what `explorer.py :: sweep_heading()` already assumes.

---

## PIN MAP — final

| Pin | Use |
|---|---|
| D0 | motor 1 |
| D1 | motor 2 |
| D2 | XSHUT — ToF#2 |
| D3 | motor 3 |
| D4 | SDA — all sensors |
| D5 | **DEAD — do not use** |
| D6 | XSHUT — ToF#3 |
| D7 | XSHUT — ToF#4 |
| D8 | SCL — all sensors |
| D9 | motor 4 |
| D10 | spare (5th laser, downward, if you add one) |

---

## STAGE 1 — SENSORS

### Already working, do not touch
MPU6050 at `0x68`: `VCC`→3V3, `GND`→GND, `SDA`→D4, `SCL`→D8.

### A. XIAO to the first laser — 4 wires

| From | To |
|---|---|
| XIAO `3V3` | ToF#1 `VCC` |
| XIAO `GND` | ToF#1 `GND` |
| XIAO `D4` | ToF#1 `SDA` |
| XIAO `D8` | ToF#1 `SCL` |

### B. Daisy chain — 12 wires

Same four signals hop board to board. Never cross them.

| From | To |
|---|---|
| ToF#1 `VCC` `GND` `SDA` `SCL` | ToF#2, same four |
| ToF#2 `VCC` `GND` `SDA` `SCL` | ToF#3, same four |
| ToF#3 `VCC` `GND` `SDA` `SCL` | ToF#4, same four |

### C. XSHUT — 3 wires, each on its own

| From | To |
|---|---|
| XIAO `D2` | ToF#2 `XSHUT` |
| XIAO `D6` | ToF#3 `XSHUT` |
| XIAO `D7` | ToF#4 `XSHUT` |

### Leave unconnected
- ToF#1 `XSHUT` — keeps the default address
- `GPIO1` on all four boards

### Board assignment — label before soldering

| Board | Beam points | XSHUT | Address |
|---|---|---|---|
| ToF#1 | **FRONT** | none | 0x30 |
| ToF#2 | **RIGHT** | D2 | 0x31 |
| ToF#3 | **BACK** | D6 | 0x32 |
| ToF#4 | **LEFT** | D7 | 0x33 |

Four beams at exactly 90°, level with each other, horizontal, mounted out past
the prop discs. The symmetry matters — `localize.py` assumes the beams sit
where the code says they sit.

Beam leaves the face with the small silver chip with two tiny windows.
Confirm each board with the finger test before mounting.

### Two wires, one hole
Boards #1–#3 have a wire arriving AND leaving on the same pad. Twist the two
bare ends together, tin as one strand, push through as a single wire.

**19 wires, 38 joints. Tug-test each as you go. Scan after every board.**

---

## STAGE 2 — MOTORS  (props off, USB power)

| From | To |
|---|---|
| XIAO `D0` | DRV8833 #1 `IN1` → motor 1 |
| XIAO `D1` | DRV8833 #1 `IN3` → motor 2 |
| XIAO `D3` | DRV8833 #2 `IN1` → motor 3 |
| XIAO `D9` | DRV8833 #2 `IN3` → motor 4 |
| both drivers `IN2`, `IN4` | `GND` |
| both drivers `EEP` | `3V3` |
| both drivers `ULT` | nothing |
| motor wires | `OUT1`/`OUT2`, `OUT3`/`OUT4` |

Motor direction: adjacent motors spin opposite, diagonals match.
Red+blue leads = CW, black+white = CCW.

---

## STAGE 3 — BATTERY

| From | To |
|---|---|
| LiPo `+` | XIAO `BAT+` pad (underside), both driver `VCC` |
| LiPo `-` | XIAO `BAT-` pad, both driver `GND`, common ground |

Do NOT feed the LiPo into the `5V` pin — use the BAT pads, they are designed
for a 1S cell and the 5V pin has too little headroom when the pack sags.

Motors draw straight from the pack, never through the XIAO.

**Never solder with the battery connected. Check polarity twice — backwards
destroys the XIAO instantly. Do not discharge below 3.3 V.**
