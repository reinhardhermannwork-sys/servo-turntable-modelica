# Roadmap

| Step | What | Status |
|---|---|---|
| D0 | kinematic CiA 402 drive: state machine, modes 1/3/6, StatusWord, ramps | done |
| D1 | inside a machine model behind the field-bus process data (FMU, co-simulation) | done |
| D2 | controller's real motor code against the FMU; sampled servo loop after the CVODE overshoot | done in the model; full machine runs pending |
| T0 | fit tool: segments, friction table, inertia, torque limit, loop gains from a 10 Hz log ([identification.md](identification.md)) | planned |
| T1 | `Mechanical` package: ramp generator, PI velocity loop with acceleration feed-forward and torque limit, turntable J·dω/dt = τ − τf(ω) − τfault; outputs torque, position, StatusWord bits 3/10/11 | planned |
| T2 | swap the kinematic drive for T1 in the machine model, compare against the log | planned |
| T3 | faults ([faults-and-supervision.md](faults-and-supervision.md) §3) + controller supervision layer + tests | planned |

Open questions that only the real drive answers: motor rating (counts → N·m), the drive setup file (ramps, loop
gains, tracking window, whether a following error trips the drive), vibration sensor levels.
