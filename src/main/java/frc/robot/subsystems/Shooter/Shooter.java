// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  ShooterIOInputs inputs = new ShooterIOInputs();

  double num =0;

  //SmartDashBoard Tuning
  double kP;
  double kI;
  double kD;

  public Shooter(ShooterIO io) {
    this.io = io;

    kP = ShooterConstants.KP_SIM;
    kI = ShooterConstants.KI_SIM;
    kD = ShooterConstants.KD_SIM;

    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    Logger.recordOutput("Shooter velocity", inputs.velocity);
    Logger.recordOutput("Left Trigger x Max Velocity??", num/ ShooterConstants.GEAR_RATIO);
    
    Logger.recordOutput("P Gain", ShooterConstants.KP_SIM);
    Logger.recordOutput("I Gain", ShooterConstants.KI_SIM);
    Logger.recordOutput("D Gain", ShooterConstants.KD_SIM);
    
    /*  PID Tuning */
    //    Gets Values from SmartDashBoard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);

    //  If the Value Changes Update the PID Values
    if((p != kP)) { ShooterConstants.KP_SIM = p; kP = p; }
    if((i != kI)) { ShooterConstants.KI_SIM = i; kI = i; }
    if((d != kD)) { ShooterConstants.KD_SIM = d; kD = d; }
    

  }

  public Command setVelocity(DoubleSupplier vel){
    return run(()->{
      io.setVelocity(vel.getAsDouble(), inputs);
      num = vel.getAsDouble();
    });
  }

}
