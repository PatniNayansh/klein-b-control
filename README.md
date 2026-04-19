# KLEIN-B — control system

KLEIN-B is a solar-powered autonomous boat for cleaning lakes that have gone eutrophic — too much nitrogen runoff, algal blooms, fish kills, the whole deal. The full project combines mechanical surface skimming with biological remediation: planting Azolla to lock up nutrients, releasing Daphnia to eat the algae.

This repo is the brain. It's the Python code that turns sensor data into thruster commands. No GUI, no simulator, no fake sensor data — real hardware drivers plug into the sensor classes.

## How it fits together

```
  Brain.tick()
      |
      v
  sense → locate → map → safety check → pick task → plan → drive
```

Each stage is its own module. `Brain` just ties them together and runs the loop.

```
klein_b/
  core/         types, world state, safety rules
  sensors/      sensor interfaces, fusion
  navigation/   localization, mapping, pathfinding, motor control
  scheduler/    task queue with priorities
  brain.py      ties everything together
```

## Implemented

- **A\* pathfinding** for getting from A to B around obstacles
- **Occupancy grid** that tracks where obstacles are, based on lidar returns
- **Zigzag (lawnmower) coverage** for cleaning the whole lake surface
- **Waypoint follower** that turns a target point into left/right motor thrust
- **Task scheduler** with priorities and preemption
- **Safety checks** — low battery, obstacle too close, boat tilting too much, staying inside the lake bounds


## Running it

```bash
pip install -e ".[dev]"

ruff check klein_b tests
mypy --strict klein_b
pytest -v

python examples/brain_wiring.py   # instantiates the Brain, doesn't actually run it
```

CI runs all four on every push.

`examples/brain_wiring.py` shows how everything gets wired up. Swap the `Unconnected*` sensor classes for real drivers and `brain.tick(dt, now)` will actually produce thrust commands.

## Next

- Hardware drivers when the parts arrive (Velodyne lidar, uBlox GPS, BNO085 IMU, the battery board, Atlas Scientific water probe)
- Finish the GPS side of the position estimator once I can test against the real receiver
- Phase-specific tasks (`DockAndCharge`, `AzollaBroadcast`, `DaphniaRelease`, etc.) in a separate mission-runner repo so this one stays generic
- Radio bridge (MQTT or LoRa) for shore telemetry and a remote abort button
