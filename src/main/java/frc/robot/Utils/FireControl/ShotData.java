// Copyright (c) 2026 Bensalem High School Fightin' Robotic Owls
// https://github.com/FRO5401
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.Utils.FireControl;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public record ShotData(double rps, double hoodRotations, double tof) {

  public ShotData(AngularVelocity flywheelVelocity, Angle hoodAngle, Time tof) {
    this(flywheelVelocity.in(RotationsPerSecond), hoodAngle.in(Rotations), tof.in(Seconds));
  }

  public ShotData(double rps, double tof) {
    this(rps, 0, tof);
  }

  public ShotData(AngularVelocity flywheelVelocity, Time tof) {
    this(flywheelVelocity.in(RotationsPerSecond), 0, tof.in(Seconds));
  }

  public static Interpolator<ShotData> interpolate() {
    return (start, end, t) ->
        new ShotData(
            MathUtil.interpolate(start.rps, end.rps, t),
            MathUtil.interpolate(start.hoodRotations, end.hoodRotations, t),
            MathUtil.interpolate(start.tof, end.tof, t));
  }

  public AngularVelocity getFlywheelVelocity() {
    return RotationsPerSecond.of(rps);
  }

  public Angle getHoodAngle() {
    return Rotations.of(hoodRotations);
  }

  public Time getToF() {
    return Seconds.of(tof);
  }

  public static final ShotData ZEROED = new ShotData(0, 0, 0);
}
