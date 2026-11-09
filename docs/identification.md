# Identifying the drive from a log (method, example numbers)

Status: method worked out on a 1-hour, 10 Hz log of a real machine (≈ 100 speed set-point changes, 10–500 rpm,
one homing + positioning sequence); the fit tool is not built yet. **All numbers below are made-up examples** with
the same shape as the real ones.

Signals needed: target and actual velocity, actual position, torque actual value (0x6077), StatusWord, position
loop (following) error, mode. Torque is in per mille of rated torque; without the motor rating plate keep it in
counts — N·m only matter for a physical inertia, not for fault tracing.

## 1. Units
Check `perRpm` on a steady high speed: actual/target should be 1.0000 within 1e-4. A rounded factor (e.g. "÷ 2,700")
shows as a constant speed error.

## 2. Ramps
Slope of actual speed during set-point changes, separately up and down. They are drive parameters (not in the
controller's commands). Example: 30 rpm/s up, 25 rpm/s down.

## 3. Steady friction
Windows of 2 s with target reached (StatusWord 0x400) and |a| < 1.5 rpm/s; mean torque per speed:

| rpm | 10 | 20 | 60 | 150 | 500 |
|---|---|---|---|---|---|
| torque (counts, example) | 20 | 35 | 50 | 65 | 90 |

Expect noise of ±10–40 counts; ±2 rpm speed reading noise at standstill is normal for a resolver.

## 4. Inertia
On ramps fit torque = J·a + c0 + c1·rpm over 2 s windows. **Use up and down ramps together:** each direction has a
constant a, so J and c0 cannot be separated from one direction alone. Example: J ≈ 50 counts per rpm/s. If the
friction on ramps is far above the steady table at high speed, suspect liquid on the table being accelerated or the
drive's feed-forward.

## 5. Torque limit and velocity loop
Look for a braking ramp interrupted by a high set point: the ramp reverses, torque hits the limit for a frame
(StatusWord bit 11 "internal limit active") and the following error jumps. Those events plus the overshoot at the
end of big ramps (≈ 1–2 %, settling ≈ 2 s) fix the PI gains and the limit.

## 6. Homing and positioning
From the StatusWord sequence: homing speed (position slope), time to the mark, final position after homing (≈ 0),
positioning time and overshoot past the target. These set `homingRpm`, `startPosition` (for a first homing like the
log's), `kp` and `window`.
