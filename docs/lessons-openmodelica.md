# Lessons (OpenModelica 1.2x, FMU export, CVODE)

Found while putting the drive into a ~200-state machine model exported as a co-simulation FMU.

1. **Continuous ramps + CVODE in a big model overshoot.** First version: speed and position as continuous states,
   speed command switching at the target. Alone (2 states) a mode-1 move stopped within 4 counts. Inside the
   machine model (199 states, CVODE, no root finding in the FMU export) the speed did not follow the braking ramp
   at the target: it drifted for ~2 s, the turntable passed the target by ~1000 counts (≈ 10 % of a turn) and came
   back, so every positioning took ~60 % longer. CVODE's error norm over all states hides the kink of one state.
   A fixed-step Euler run of the same model had no issue. **Fix: sample the servo loop** (discrete states, 10 ms,
   like a real drive) → overshoot 4 counts again, independent of the solver. `Tests.NoOvershoot` guards it.
2. **Don't put sample instants on the FMU communication grid.** Updates at multiples of 50 ms coincided with the
   communication points and CVODE stopped with "too close" (−27) at t = 0.1. `sample(Ts/2, Ts)` (half a period
   off) fixed it.
3. **A piecewise-constant output broke the FMU code generation** (`_omcQ_24DER…` undeclared: the export wants output
   derivatives). Interpolating the position between updates keeps it continuous and fixed the build.
4. **`integer(realInput)` in front of when-clauses blew up omc's FMU export** (C code generation > 2 GB, no end;
   the plain simulation was fine). Declaring ControlWord and mode as Integer inputs end to end built in 20 s.
   Also an extra `when` inside the algorithm section had the same effect once. Put a timeout on FMU builds, and
   note that killing `docker run` does not stop the container — remove it by name.
5. **Model what the real drive does on mode switches.** Entering mode 1 must not start a move by itself.
6. **Compare runs on unique time stamps.** Event iterations write several rows with the same time into the result
   file; matching rows by time gave dozens of false differences.
