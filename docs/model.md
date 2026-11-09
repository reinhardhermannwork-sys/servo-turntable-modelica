# The drive model

## 1. Signals

| CiA 402 object | Direction | Block port | Unit |
|---|---|---|---|
| 0x6040 ControlWord | in | `controlWord` | — |
| 0x6060 Modes of operation | in | `modeOfOperation` | 1, 3, 6 |
| 0x60FF Target velocity | in | `targetVelocity` | 0.1 counts/s |
| 0x607A Target position | in | `profileTargetPosition` | counts |
| 0x6081 Profile velocity | in | `profileTargetVelocity` | 0.1 counts/s |
| 0x607C Home offset | in | `homingOffset` | counts |
| 0x6041 StatusWord | out | `statusWord` | — |
| 0x6061 Modes of operation display | out | `modeDisplay` | — |
| 0x6064 Position actual value | out | `actualPosition` | counts |
| 0x606C Velocity actual value | out | `actualVelocity` | 0.1 counts/s |

The ramp rates are **drive parameters**: the controller sends only the target speed. A ramp changed in the drive
setup therefore shows on the machine without any change on the controller side — worth knowing when comparing runs.

## 2. State machine and StatusWord

Low byte from `ControlWord & 15`: 15 → 0x37 operation enabled, 7 → 0x33 switched on, 6 → 0x31 ready to switch on,
anything else → 0x70 switch-on disabled. Fault (bit 3) is not modelled yet.

Added bits: 0x200 remote (always), 0x400 target reached, 0x1000 mode-specific bit 12, 0x4000 moving
(|v| > `vRest`; manufacturer-specific bit, common on servo drives).

| Value | Meaning |
|---|---|
| 0x1270 | switch-on disabled (ControlWord 0) |
| 0x0637 | enabled, target reached, bit 12 clear (mode 1 at rest, or mode 6 before homing) |
| 0x4237 | enabled, moving, not reached (homing, or a position move after the handshake) |
| 0x5237 | moving + bit 12 (mode 1 set-point acknowledge, or mode 3 ramping) |
| 0x5637 | mode 3 at the target speed while turning |
| 0x1637 | at rest + bit 12 (mode 3 stopped, or mode 6 homing attained) |

## 3. Modes

- **Mode 3, profile velocity:** speed ramps to `targetVelocity` at `accelRpm` up / `decelRpm` down, with a short
  first-order approach (`tau`) at the end of the ramp. Bit 12 always set, target reached within `speedWindowRpm`.
- **Mode 6, homing:** a rising ControlWord bit 4 starts it; the turntable turns at −`homingRpm` to the next index
  mark below (raw position ≡ 0 mod `countsPerRev`); there the position becomes `homingOffset` and the motor stops:
  0x1637. Entering mode 6 clears "attained". The first homing time is set by `startPosition`.
- **Mode 1, profile position:** a rising bit 4 takes target and profile velocity; bit 12 = acknowledge while bit 4
  is high; speed = clamp(10·kp·(target − position), ±profile velocity); target reached within `window` counts and
  nearly at rest. **Entering mode 1 without a new set point does not move** — a real drive holds its position
  (an early version headed for the old target on every mode switch).
- **Disabled:** speed falls at the braking ramp (no brake model yet).

## 4. A typical controller sequence (what `Tests.DriveSequence` replays)

| Step | Controller writes | Drive answers |
|---|---|---|
| enable | mode 3, ControlWord 15 | 0x1637 |
| spin | TargetVelocity = rpm·perRpm | 0x5237 while ramping, 0x5637 at speed, 0x1637 stopped |
| home | mode 1 → display 1; mode 6 → display 6; ControlWord 15 → 31 | 0x4237 while homing, then 0x1637 attained |
| position | mode 1, profile velocity + target, ControlWord 15 → 31 → 15 | 0x5237 ack, 0x4237 moving, 0x0637 reached |
| disable | ControlWord 0 | 0x1270 |

Controllers typically wait for "bit 12 clear" after starting homing, then for `SW & 0x5400 == 0x1400`
(attained + reached + not moving), and treat bit 13 (0x2000) as a homing/following error. The mode display must
follow the mode quickly: controllers often busy-wait on it.

## 5. Sampling

Speed `v`, raw position `x` and the speed command are discrete and updated every `Ts` (10 ms) in
`when sample(Ts/2, Ts)`: ramp limit `accel·Ts` or `decel·Ts`, approach factor `Ts/tau`, trapezoidal position step,
homing mark and mode-1 end checked inside the update. The position output interpolates between updates
(`x + v·(time − tUpdate)/10`). Why: [lessons-openmodelica.md](lessons-openmodelica.md).
