# KLEIN-B — control system

KLEIN-B is a solar-powered autonomous boat for cleaning lakes that have gone eutrophic — too much nitrogen runoff, algal blooms, fish kills, the whole deal. The full project combines mechanical surface skimming with biological remediation: Azolla planting to lock up nutrients, Daphnia release to eat the algae.

This repo is the brain. It's the Python package that turns sensor data into thruster commands. No GUI, no simulator, no mock sensor data — hardware drivers plug into the sensor ABCs.

## How it fits together

```
  Brain.tick()
      |
      v
  sense → fuse → localize → map → safety → schedule → plan → control
```

Each stage is its own module. `Brain` wires them together and runs the loop.

```
klein_b/
  core/         types, world/boat state, safety monitor + rules
  sensors/      abstract sensor interfaces, fusion
  navigation/   localization (EKF), occupancy grid, A*, boustrophedon, controller
  scheduler/    task ABC, priority scheduler
  brain.py      composes everything
```

## Implemented

- **A\* path planner** — 8-connected, octile heuristic
- **Log-odds occupancy grid** with Bresenham raycasting for lidar integration
- **Boustrophedon coverage** for full-surface sweeps (skimmer passes, Azolla broadcast)
- **Pure-pursuit controller** with differential-thrust mixing
- **Priority scheduler** with preemption and safety-interrupt handling
- **Safety monitor** — low battery, obstacle proximity, capsize (IMU tilt), geofence

## Stubbed

- **Sensor drivers.** Each sensor (lidar, GPS, IMU, battery, water probe) is an ABC. Real drivers subclass these. Mock implementations would hide unit and frame-of-reference bugs, which is what the ABCs exist to catch.
- **GPS update step in the EKF.** State vector, covariance, predict step, and IMU update are done. The Kalman gain for GPS is a single TODO — picking `R_gps` needs real receiver noise data, not a guess.

## Running it

```bash
pip install -e ".[dev]"

ruff check klein_b tests
mypy --strict klein_b
pytest -v

python examples/brain_wiring.py   # instantiates the Brain, doesn't tick
```

CI runs all four on every push.

`examples/brain_wiring.py` shows how everything gets wired. Swap the `Unconnected*` sensor classes for real drivers and `brain.tick(dt, now)` will produce thrust commands.

## Next

- Hardware drivers once the parts arrive (Velodyne, uBlox ZED-F9P, BNO085, BMS, Atlas Scientific EZO)
- Characterize GPS covariance, finish the EKF update
- Phase-specific tasks (`DockAndCharge`, `AzollaBroadcast`, `DaphniaRelease`, etc.) in a separate mission-runner repo so this one stays generic
- MQTT or LoRa bridge for shore telemetry and remote abort
