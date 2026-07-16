// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Copyright (c) 2026 Bensalem High School Fightin' Robotic Owls
// https://github.com/FRO5401
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.subsystems.Hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HoodConstants.HoodMode;
import frc.robot.Utils.FireControl.ShotData;
import frc.robot.Utils.Tunable.TunableNumber;
import frc.robot.Utils.Tunable.TunableShotData;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private HoodMode hoodMode = HoodMode.Dynamic;

  private Supplier<Pose2d> robotPose;
  private Supplier<Boolean> isIntakeDeployed;

  private TunableNumber kp = new TunableNumber("Hood/kp", HoodConstants.KP);
  private TunableNumber ki = new TunableNumber("Hood/ki", HoodConstants.KI);
  private TunableNumber kd = new TunableNumber("Hood/kd", HoodConstants.KD);
  private TunableNumber ks = new TunableNumber("Hood/ks", HoodConstants.KS);

  private TunableShotData shotData = new TunableShotData("ShotData", ShotData.ZEROED);

  private double desiredPosition = 0;

  /** Creates a new Hood. */
  public Hood(HoodIO io, Supplier<Pose2d> robotPose, Supplier<Boolean> isIntakeDeployed) {
    this.io = io;
    this.robotPose = robotPose;
    this.isIntakeDeployed = isIntakeDeployed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    if (kp.hasChanged() || ki.hasChanged() || kd.hasChanged() || ks.hasChanged()) {
      io.setPID(kp.get(), ki.get(), kd.get(), ks.get());
    }

    if (shotData.hasChanged(hashCode())) {
      Logger.recordOutput("Hood/Changed", true);
    }
    Logger.recordOutput("ShotData/Values", shotData.getValue());

    // if(isIntakeDeployed.get() && hoodMode.equals(HoodMode.Dynamic)){
    //   io.setPosition(0);
    // }

    Logger.recordOutput("Hood/Disired Angle", desiredPosition);
  }

  public Command setHoodCommand(double position) {
    return Commands.runOnce(() -> setHoodPosition(position));
  }

  public Command setHoodCommand(ShotData data) {
    return Commands.runOnce(() -> setHoodPosition(data.rps()));
  }

  public void setHoodPosition(double position) {
    position = MathUtil.clamp(position, 0, 0.53125);
    desiredPosition = position;
    io.setPosition(position);
  }

  public Command setHoodCommandwIntake(DoubleSupplier hoodPosition, DoubleSupplier intakePose) {

    return runOnce(
        () -> {
          if (intakePose.getAsDouble() != 0) {
            setHoodPosition(hoodPosition.getAsDouble());
          } else {
            setHoodPosition(0);
          }
        });
  }

  public Command setHoodCommandwIntake(Supplier<ShotData> hoodPosition, DoubleSupplier intakePose) {

    return runOnce(
        () -> {
          if (intakePose.getAsDouble() != 0) {
            setHoodPosition(hoodPosition.get().hoodRotations());
          } else {
            setHoodPosition(0);
          }
        });
  }
}
