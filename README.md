# <name it>

my pet phantom

![assembled](docs/photos/assembled.jpeg)

## What it does
so for now the drone only hovers but with funding im planning to add the phantom capability which basically makes the rear motors turn 90 degrees back acting as pure thruster while the front do the heavy lifting and can cruise at 35mph and 46mph short bursts

## Why I built it
i used to have a bird when i was a kid called mango but after a year it died so the reason im making this is so this time i create a sarcastic pet that when destroyed i can just upload its consiousnesss to another body

## How it works
the drone works by 3 crucial components the gyroscope which gives a number the ai has to calculate so it can correct itself mid air 100 times a second the TOF sensors which show the distance and stuff if possible with funding the steval vl53l9 lidar and the battery ofcourse
the tof require separate xhut pins so both of them show different reading which i confused first and the main THE ABSOLUTE GOAT the ESP 32 S3 has inbuilt BLE WIFI over 8mb psram all under 2grams!!!

## Wiring
See [docs/WIRING.md](docs/WIRING.md).
![wiring](docs/photos/wiring.jpeg)

## Firmware
| file | what it does |
|---|---|
| `firmware/drone_slam.ino` | main - sensors, heading, occupancy grid, UDP telemetry |
| `firmware/imu_bringup.ino` | MPU6050 bring-up with a full I2C bus scan |
| `firmware/tof_bringup.ino` | re-addresses all four ToF sensors off 0x29 |
| `firmware/cam_diag.ino` | camera fault diagnosis |

## Software
<!-- occupancy.py, explorer.py, localize.py, sim.py - one line each -->

## Bill of materials
See [BOM.csv](BOM.csv).

## What I got wrong
.
     - four solder joints that held mechanically and conducted nothing
     - motors that only crackled - IN2/IN4 floating on the second driver
     - the gyro reading zeros because of a bridge to the metal shield
     - cutting the header pins with craft scissors

## Status / next
i still got like 3 more joints until it flies and tbh im just being lazy 
