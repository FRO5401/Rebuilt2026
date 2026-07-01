// Copyright (c) 2026 Bensalem High School Fightin' Robotic Owls
// https://github.com/FRO5401
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotMode {

  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  // should tuning be on when not connected to FMS
  public static final boolean isTuningOff = false;
  public static final boolean isTuningMode = !(DriverStation.isFMSAttached() || isTuningOff);
}
