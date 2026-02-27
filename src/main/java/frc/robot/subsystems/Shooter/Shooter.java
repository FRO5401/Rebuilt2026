// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private ShooterIOInputs inputs = new ShooterIOInputs();

  private double desiredVel = 0;

  public Shooter(ShooterIO io) {
    this.io = io;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    Logger.recordOutput("Shooter/Shooter velocity", getVelocity().in(RotationsPerSecond));
    Logger.recordOutput("Shooter/Shooter Desired Velocity", desiredVel);

  }

  public Command setVelocity(Supplier<AngularVelocity> vel, Supplier<Double> intakePose) {

    return runOnce(() -> {
      if (intakePose.get() != 0) {
        io.setVelocity(vel.get().in(RotationsPerSecond),inputs);
        desiredVel = vel.get().in(RotationsPerSecond);
      } else {
        io.setVelocity(0);
        desiredVel = 0;
      }
    });

  }

  public AngularVelocity getVelocity() {
    return RotationsPerSecond.of(inputs.velocity);
  }

  public void stop() {
    io.stop();
  }

}
