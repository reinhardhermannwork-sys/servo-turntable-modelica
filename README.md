# ServoTurntable — a CiA 402 servo drive and turntable in Modelica

A small Modelica library for one thing: a **servo-driven turntable as a field-bus controller sees it**. The block
`ServoTurntable.Drive` takes the CiA 402 (CANopen / EtherCAT CoE drive profile) commands a controller writes and
answers with the StatusWord, mode display, position and velocity a real drive would report:

- device state machine (switch-on disabled → ready → switched on → operation enabled),
- **mode 3** profile velocity with drive-side acceleration / braking ramps,
- **mode 6** homing to the next index mark in negative direction (method 33 style), with homing offset,
- **mode 1** profile position with set-point handshake (ControlWord bit 4 ↔ StatusWord bit 12) and target window,
- StatusWord bits remote, target reached, bit 12 (speed / homing attained / set-point acknowledge) and moving.

It was written to plug a real machine controller into a digital twin: the controller's own motor code runs
unchanged against the model instead of a built-in fake, so homing, positioning and spin ramps take the time they
take on the machine. The servo loop is **sampled** (10 ms), which makes the block solver-independent — the most
important lesson we learnt (see [docs/lessons-openmodelica.md](docs/lessons-openmodelica.md)).

All numbers in this repository (resolution, ramps, speeds, friction tables) are **made-up examples**. Set the
parameters from your own drive configuration and measurements ([docs/identification.md](docs/identification.md)).

## Quick start

```
sh test.sh        # needs Docker; runs the tests in openmodelica/openmodelica:v1.27.1-ompython
```

```modelica
ServoTurntable.Drive drive(countsPerRev = 10000, accelRpm = 30, decelRpm = 25, homingRpm = 3);
equation
  drive.controlWord = controlWord;          // from your controller / FMU inputs
  drive.modeOfOperation = mode;
  drive.targetVelocity = rpm*drive.perRpm;  // 0.1 counts/s
  ...
```

Units: positions in encoder counts, velocities in 0.1 counts/s (`perRpm = countsPerRev·10/60` per rpm).

## Contents

| Path | What |
|---|---|
| `ServoTurntable/package.mo` | `Drive` block, `Tests.DriveSequence` (18 checks along a full controller sequence), `Tests.NoOvershoot` |
| `test.sh` | runs the tests in the OpenModelica container, PASS/FAIL per test |
| [docs/model.md](docs/model.md) | state machine, StatusWord values, modes, the controller sequence the block answers |
| [docs/lessons-openmodelica.md](docs/lessons-openmodelica.md) | what went wrong on the way and why the loop is sampled (CVODE, FMU export) |
| [docs/identification.md](docs/identification.md) | how to get ramps, friction, inertia, torque limit and loop gains from a 10 Hz drive log |
| [docs/faults-and-supervision.md](docs/faults-and-supervision.md) | fault signatures in the drive signals, and which supervision really runs |
| [docs/roadmap.md](docs/roadmap.md) | what is built, what is planned (torque / friction / inertia model, faults) |

## Status

Kinematic drive model, tested (2 tests, OpenModelica 1.27.1, Modelica Standard Library 4.1.0). Used inside a
~200-state machine model exported as a co-simulation FMU (CVODE). Torque, friction, inertia and drive faults are
designed but not built yet ([docs/roadmap.md](docs/roadmap.md)).

## License

MIT, see [LICENSE](LICENSE).
