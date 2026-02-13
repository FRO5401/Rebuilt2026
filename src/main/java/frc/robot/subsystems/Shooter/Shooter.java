// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  ShooterIOInputs inputs = new ShooterIOInputs();

  // Checks if the robot is real or fake, and uses the correct PID controller
  PIDController controller = RobotBase.isReal()
      ? new PIDController(ShooterConstants.KP, ShooterConstants.KI, ShooterConstants.KD)
      : new PIDController(ShooterConstants.KP_SIM, ShooterConstants.KI_SIM, ShooterConstants.KD_SIM);

  TalonFX motor;

  public Shooter(ShooterIO io) {
    controller.enableContinuousInput(0, 1);
    motor = new TalonFX(ShooterConstants.MOTOR_ID);
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    io.updateInputs(inputs);
    io.applyVoltage(controller.calculate(inputs.voltage));
    
    setVelocity(inputs.velocity);
  
  }

  public void setVelocity(double vel){
    motor.set(vel);
  }
}
