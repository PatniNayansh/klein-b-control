# KLEIN-B — control system

KLEIN-B is a solar-powered autonomous boat for cleaning lakes that have gone
eutrophic (too much nitrogen runoff, algal blooms, fish kills, the whole deal).
The full project combines mechanical surface skimming with biological remediation
— Azolla planting to lock up nutrients, Daphnia release to eat the algae.

This repo is the **brain** — the Python package that turns sensor data into
thruster commands. No GUI, no simulator, no fake sensor data. The hardware
drivers plug in where the sensor ABCs are.

## How it fits together

```
  Brain.tick()
      |
      v
  sense -> fuse -> localize -> map -> safety -> schedule -> plan -> control
```

Each stage is its own module under `klein_b/`. `Brain` just wires them
together and runs the loop.

```
klein_b/
  core/         types, world/boat state, safety monitor + rules
  sensors/      abstract sensor interfaces, fusion
  navigation/   localization (EKF), occupancy grid, A*, boustrophedon, controller
  scheduler/    task ABC, priority scheduler
  brain.py      composes everything
```

## What's actually implemented

The algorithms are real code, not stubs:

- **A\* path planner** (8-connected, octile heuristic) in `navigation/path_planner.py`
- **Log-odds occupancy grid** with Bresenham raycasting for lidar integration
- **Boustrophedon coverage** for full-surface sweeps (skimmer passes, Azolla broadcast)
- **Pure-pursuit waypoint controller** with differential-thrust mixing
- **Priority task scheduler** with preemption and safety-interrupt handling
- **Safety monitor** with four rules: low battery, obstacle proximity, capsize (from IMU tilt), geofence

## What's stubbed (on purpose)

- **Sensor drivers.** Each sensor (lidar, GPS, IMU, battery, water probe) is an
  ABC with `NotImplementedError` on `read()`. Real drivers subclass these.
  I didn't fake them because the whole point of having the ABCs is that a
  fake version would paper over the units/frame mistakes you actually care about.

- **GPS measurement update in the EKF.** The state vector, covariance, predict
  step, and IMU-based update are all implemented. The Kalman gain block for
  GPS is a single TODO in `localization.py`. I need real receiver data to
  pick R_gps before that's worth writing — hardcoding made-up numbers isn't
  actually an EKF, it's just a fancier way of lying.

## Running it

```bash
pip install -e ".[dev]"

ruff check klein_b tests
mypy --strict klein_b
pytest -v

python examples/brain_wiring.py   # instantiates the Brain, doesn't tick
```

CI runs all four on every push.

`examples/brain_wiring.py` is the quickest way to see how everything gets
wired up. Replace the `Unconnected*` sensor classes with real drivers and
`brain.tick(dt, now)` will produce actual thrust commands.

## Next steps

- Write the hardware drivers once I have the parts in hand (Velodyne, uBlox
  ZED-F9P, BNO085, the BMS, Atlas Scientific EZO)
- Characterize GPS covariance and finish the EKF measurement update
- Build the phase-specific tasks (`DockAndCharge`, `AzollaBroadcast`,
  `DaphniaRelease`, etc.) in a separate mission-runner repo so this one
  stays generic
- MQTT or LoRa bridge so shore can watch telemetry and hit an abort button

## License

MIT — see [LICENSE](./LICENSE).
