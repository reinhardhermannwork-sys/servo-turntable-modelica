# Faults and supervision

## 1. Fault signatures (what the drive signals show)

| Fault | Expected in the log |
|---|---|
| bearing / seal wear, fluid drag | steady torque up at the same speed (friction curve shifts), ramps unchanged |
| brake not released / dragging | constant extra torque from 0 rpm on; at the limit the ramp gets slower |
| extra or unbalanced load | more torque on ramps (inertia), vibration warning/alarm at high speed |
| seized turntable or motor | torque at the limit, speed ≈ 0 — possibly **no controller alarm** (§2) |
| drive trip (amplifier, over-temperature, feedback lost) | StatusWord bit 3 (fault) |
| motor supply lost | drive disabled, turntable coasts down |

## 2. Which supervision really runs — check it

Typical controller-side checks: poll StatusWord bit 3 (fault) every few hundred ms and abort; vibration alarm →
slow down to a safe speed and abort the process; index pin not retracted while turning → abort.

Lesson: a controller may contain speed-tolerance / no-rotation / acceleration checks in its source that are **only
called by its built-in simulated motor**, not on the real drive path (and a tolerance parameter can switch the rest
off). Then a slow, stuck or wrong-speed turntable raises no controller alarm on the real machine; only the drive's
own protection (I²t, following-error trip, over-temperature — set in the drive configuration) or a vibration sensor
stops the process. When you build a twin, run the controller against the drive model (not its own fake) so that the
supervision you test is the one that runs on the machine.

## 3. Planned fault parameters
Default off, each with an onset time and a size: friction factor, drag torque (brake), inertia factor (load),
unbalance (vibration ∝ unbalance·ω², warning/alarm thresholds as parameters), drive fault, feedback lost, seized
(torque at limit, speed 0), motor supply. See [roadmap.md](roadmap.md).
