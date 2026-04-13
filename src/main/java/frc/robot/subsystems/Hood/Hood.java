// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Hood;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HoodConstants.HoodMode;
import frc.robot.Constants.TurretConstants.TurretMode;
import frc.robot.Utils.TunableNumber;
import frc.robot.subsystems.Hood.HoodIO.HoodIOInputs;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private HoodMode hoodMode = HoodMode.Dynamic;

  private Supplier<Pose2d> robotPose;
  private Supplier<Boolean> isIntakeDeployed;

  private TunableNumber kp = new TunableNumber("Hood/kp", HoodConstants.KP);
  private TunableNumber ki = new TunableNumber("Hood/ki", HoodConstants.KI);
  private TunableNumber kd = new TunableNumber("Hood/kd", HoodConstants.KD);
  private TunableNumber kff = new TunableNumber("Hood/kff", HoodConstants.KFF);

  /** Creates a new Hood. */
  public Hood(HoodIO io, Supplier<Pose2d> robotPose, Supplier<Boolean> isIntakeDeployed) {
    this.io = io;
    this.robotPose = robotPose;
    this.isIntakeDeployed = isIntakeDeployed;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood/", inputs);

    if(kp.hasChanged() || ki.hasChanged() || kd.hasChanged() || kff.hasChanged()){
      io.setPID(kp.get(), ki.get(), kd.get(), kff.get());
    }

    if(isIntakeDeployed.get() && hoodMode.equals(HoodMode.Dynamic)){
      io.setPosition(0);
    }
    
  }

  public Command setHoodCommand(double position){
    return Commands.runOnce(()->io.setPosition(position));
  }

  
  
}
