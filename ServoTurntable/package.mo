within;
package ServoTurntable "CiA 402 servo drive with a turntable, as a field-bus controller sees it: state machine, profile
  velocity (mode 3), homing on an index mark (mode 6, method 33 style) and profile position (mode 1); the servo loop is
  sampled, so the model behaves the same under any solver (docs/model.md, docs/lessons-openmodelica.md)"
  extends Modelica.Icons.Package;
  import SI = Modelica.Units.SI;

  block Drive "CiA 402 servo drive and turntable. Velocities in 0.1 counts/s, positions in counts. Speed and position
    are discrete states updated every Ts, like the servo loop of a real drive: as continuous states, the end of a ramp
    can be missed by a variable-step solver in a large model (docs/lessons-openmodelica.md, lesson 1)"
    parameter SI.Time Ts = 0.01 "servo update period";
    parameter Real countsPerRev = 10000 "encoder counts per turntable revolution (example)";
    final parameter Real perRpm = countsPerRev*10/60 "velocity units (0.1 counts/s) per rpm";
    parameter Real accelRpm = 30 "rpm/s, acceleration ramp (a drive parameter, not sent by the controller; example)";
    parameter Real decelRpm = 25 "rpm/s, braking ramp (example)";
    parameter SI.Time tau = 0.05 "approach at the end of a ramp";
    parameter Real homingRpm = 3 "homing speed, rpm (example)";
    parameter Real kp = 10 "1/s, position loop gain in mode 1";
    parameter Real window = 50 "counts: target reached in mode 1";
    parameter Real speedWindowRpm = 1 "rpm: target speed reached in mode 3";
    parameter Real vRest = 100 "0.1 counts/s: below this the motor counts as standing";
    parameter Real startPosition = 6000 "raw counts above an index mark at the start (sets the first homing time)";
    Modelica.Blocks.Interfaces.IntegerInput controlWord "CiA 402 0x6040";
    Modelica.Blocks.Interfaces.IntegerInput modeOfOperation "CiA 402 0x6060: 1, 3 or 6";
    Modelica.Blocks.Interfaces.RealInput targetVelocity "0x60FF, mode 3, 0.1 counts/s";
    Modelica.Blocks.Interfaces.RealInput profileTargetPosition "0x607A, mode 1, counts";
    Modelica.Blocks.Interfaces.RealInput profileTargetVelocity "0x6081, mode 1, 0.1 counts/s";
    Modelica.Blocks.Interfaces.RealInput homingOffset "0x607C, counts";
    Modelica.Blocks.Interfaces.IntegerOutput statusWord "0x6041";
    Modelica.Blocks.Interfaces.IntegerOutput modeDisplay "0x6061";
    Modelica.Blocks.Interfaces.RealOutput actualPosition "0x6064, counts";
    Modelica.Blocks.Interfaces.RealOutput actualVelocity "0x606C, 0.1 counts/s";
    discrete Real x(start = startPosition, fixed = true) "raw encoder position, counts";
    discrete Real v(start = 0, fixed = true) "0.1 counts/s";
    Boolean enabled = mod(controlWord, 16) == 15 "operation enabled";
    Boolean newSetPoint = mod(div(controlWord, 16), 2) == 1 "ControlWord bit 4";
    Boolean moving = abs(v) > vRest;
    discrete Real vCmd(start = 0, fixed = true) "speed command of the last servo update";
  protected
    final parameter Real accel = accelRpm*perRpm;
    final parameter Real decel = decelRpm*perRpm;
    final parameter Real homingSpeed = homingRpm*perRpm;
    discrete Boolean homing(start = false, fixed = true);
    discrete Boolean attained(start = false, fixed = true);
    discrete Boolean moveActive(start = false, fixed = true);
    discrete Real xOffset(start = 0, fixed = true) "raw position of position 0 (set by homing)";
    discrete Real xMark(start = 0, fixed = true) "raw position of the index mark homing runs to";
    discrete Real xTarget(start = 0, fixed = true) "mode 1 target (position, not raw)";
    discrete Real vProfile(start = 0, fixed = true);
    discrete Real vNew(start = 0, fixed = true), dvMax(start = 0, fixed = true);
    discrete SI.Time tUpdate(start = 0, fixed = true) "time of the last servo update";
    Boolean reached, bit12;
    Integer low;
  algorithm
    when modeOfOperation == 6 then
      attained := false;
    end when;
    when newSetPoint and modeOfOperation == 6 and enabled then
      homing := true;
      attained := false;
      xMark := countsPerRev*floor((x - 1)/countsPerRev) "next index mark below";
    end when;
    when newSetPoint and modeOfOperation == 1 and enabled then
      xTarget := profileTargetPosition;
      vProfile := abs(profileTargetVelocity);
      moveActive := true;
    end when;
    // updates half a period off the time grid: a servo update on the same instant as an FMU communication point
    // made CVODE stop with "too close" (lesson 2)
    when sample(Ts/2, Ts) then
      vCmd := if not enabled then 0
        elseif modeOfOperation == 3 then targetVelocity
        elseif modeOfOperation == 6 then (if homing then -homingSpeed else 0)
        elseif modeOfOperation == 1 then (if moveActive then max(-vProfile, min(vProfile, 10*kp*(xTarget - (x - xOffset))))
          else 0) // no set point: a real drive stays where it is when it enters mode 1 (lesson 5)
        else 0;
      // speed follows vCmd within the ramp limits (accelerating / braking) and the end-of-ramp approach tau
      dvMax := (if abs(vCmd) > abs(v) then accel else decel)*Ts;
      vNew := v + max(-dvMax, min(dvMax, (vCmd - v)*min(1, Ts/tau)));
      x := x + (v + vNew)/2*Ts/10;
      v := vNew;
      tUpdate := time;
      if homing and x <= xMark then
        homing := false;
        attained := true;
        xOffset := xMark - homingOffset;
      end if;
      if moveActive and abs(xTarget - (x - xOffset)) < window and abs(v) <= vRest then
        moveActive := false;
      end if;
    end when;
  equation
    // the encoder counts on between two updates (also keeps the output differentiable for the FMU export, lesson 3)
    actualPosition = x + v*(time - tUpdate)/10 - xOffset;
    actualVelocity = v;
    modeDisplay = modeOfOperation;
    // state machine (low byte): operation enabled 0x37, switched on 0x33, ready to switch on 0x31, else
    // switch-on disabled 0x70
    low = if mod(controlWord, 16) == 15 then 55 elseif mod(controlWord, 16) == 7 then 51
      elseif mod(controlWord, 16) == 6 then 49 else 112;
    reached = if not enabled then false
      elseif modeOfOperation == 3 then abs(v - targetVelocity) < speedWindowRpm*perRpm
      elseif modeOfOperation == 6 then not homing
      elseif modeOfOperation == 1 then not moveActive else true;
    bit12 = if modeOfOperation == 3 then true elseif modeOfOperation == 6 then attained
      elseif modeOfOperation == 1 then newSetPoint else false "speed / homing attained / set-point acknowledge";
    // + 0x200 remote, 0x400 target reached, 0x1000 mode-specific bit 12, 0x4000 moving (manufacturer bit, common)
    statusWord = low + 512 + (if reached then 1024 else 0) + (if bit12 then 4096 else 0)
      + (if moving then 16384 else 0);
    annotation(Documentation(info = "<html><p>See docs/model.md for the state machine, the StatusWord values and the
      controller sequence this block answers.</p></html>"));
  end Drive;

  package Tests "simulate each model; a failed check stops the simulation with CHECK FAILED"
    extends Modelica.Icons.ExamplesPackage;

    model Expect "check a Boolean at a given time"
      parameter SI.Time at;
      parameter Boolean expected = true;
      parameter String what = "";
      Modelica.Blocks.Interfaces.BooleanInput u;
    equation
      when time >= at then
        assert(u == expected, "CHECK FAILED: " + what + " expected " + String(expected) + " at t=" + String(at));
      end when;
    end Expect;

    model DriveSequence "a typical controller sequence (docs/model.md §4): home from 6000 counts above a mark, move
      to 4000 counts at 3 rpm, spin 12 rpm and stop, enter mode 1 again without a set point, disable"
      Drive d;
      Integer cw = if time < 1 then 0 elseif time < 1.4 then 15 elseif time < 1.6 then 31 elseif time < 15.5 then 15
        elseif time < 15.7 then 31 elseif time < 34 then 15 else 0;
      Integer mode = if time < 1 then 3 elseif time < 1.2 then 1 elseif time < 15 then 6 elseif time < 26 then 1
        elseif time < 33.7 then 3 elseif time < 34 then 1 else 3;
      Expect a1(at=0.5, what="disabled at start: 0x1270", u=d.statusWord == 4720);
      Expect s4(at=1.1, what="mode 1 display and 0x637", u=d.modeDisplay == 1 and d.statusWord == 1591);
      Expect s5(at=1.3, what="mode 6, not yet started: 0x637", u=d.modeDisplay == 6 and d.statusWord == 1591);
      Expect h1(at=2, what="homing: 0x4237", u=d.statusWord == 16951);
      Expect h2(at=2, what="homing at -3 rpm", u=abs(d.actualVelocity + 3*d.perRpm) < 50);
      Expect h3(at=12.5, what="still homing 11 s after start", u=d.statusWord == 16951);
      Expect h4(at=14.5, what="homing attained about 12 s after start: 0x1637", u=d.statusWord == 5687);
      Expect h5(at=14.5, what="position about 0 after homing", u=abs(d.actualPosition) < 60);
      Expect p1(at=15.6, what="set point acknowledged: 0x5237", u=d.statusWord == 21047);
      Expect p2(at=16, what="moving, ack clear: 0x4237", u=d.statusWord == 16951);
      Expect p3(at=22.5, what="still moving 7 s after the set point", u=d.statusWord == 16951);
      Expect p4(at=25.5, what="target reached: 0x637", u=d.statusWord == 1591);
      Expect p5(at=25.5, what="at the target 4000", u=abs(d.actualPosition - 4000) < 50);
      Expect s1(at=29, what="12 rpm reached: 0x5637", u=d.statusWord == 22071);
      Expect s2(at=29, what="speed 12 rpm", u=abs(d.actualVelocity - 12*d.perRpm) < 300);
      Expect s3(at=33.5, what="stopped in mode 3: 0x1637", u=d.statusWord == 5687);
      Expect m1(at=33.9, what="entering mode 1 again holds the position: 0x637", u=d.statusWord == 1591);
      Expect off(at=35, what="disabled: 0x1270", u=d.statusWord == 4720);
    equation
      d.controlWord = cw;
      d.modeOfOperation = mode;
      d.targetVelocity = if time > 26 and time < 31 then 12*d.perRpm else 0;
      d.profileTargetPosition = 4000;
      d.profileTargetVelocity = 3*d.perRpm;
      d.homingOffset = 0;
      annotation(experiment(StopTime=36, Interval=0.01));
    end DriveSequence;

    model NoOvershoot "a long mode-1 move must stop within the window: the check that failed with continuous states in
      a large model (lesson 1); here it guards the sampled loop against regressions"
      Drive d(startPosition = 0);
      Integer cw = if time < 0.5 then 15 elseif time < 0.7 then 31 else 15;
      Expect r(at=19, what="target 9000 reached", u=d.statusWord == 1591);
      Expect w(at=19, what="within 50 counts of 9000", u=abs(d.actualPosition - 9000) < 50);
      Real peak(start = 0, fixed = true);
      Expect o(at=19.5, what="never more than 50 counts past the target", u=peak < 9050);
    equation
      d.controlWord = cw;
      d.modeOfOperation = 1;
      d.targetVelocity = 0;
      d.profileTargetPosition = 9000;
      d.profileTargetVelocity = 5*d.perRpm;
      d.homingOffset = 0;
      when sample(0, 0.01) then
        peak = max(pre(peak), d.actualPosition);
      end when;
      annotation(experiment(StopTime=20, Interval=0.01));
    end NoOvershoot;
  end Tests;
  annotation(uses(Modelica(version = "4.1.0")), version = "0.1.0");
end ServoTurntable;
