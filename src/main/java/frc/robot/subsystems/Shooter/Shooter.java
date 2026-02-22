// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;


import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  ShooterIOInputs inputs = new ShooterIOInputs();

  double desiredVel = 0;

  public Shooter(ShooterIO io) {
    this.io = io;

    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
    ShooterConstants.TREE_MAP.put(null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    Logger.recordOutput("Shooter/Shooter velocity", inputs.velocity);
    Logger.recordOutput("Shooter/Shooter Desired Velocity", desiredVel);

  }

  public Command setVelocity(Supplier<AngularVelocity> vel){
    return run(()->{
      io.setVelocity(vel.get().in(RotationsPerSecond)*60, inputs);
      desiredVel = vel.get().in(RotationsPerSecond)*60;
    });
  }

  public AngularVelocity getVelocity(){
    return RotationsPerSecond.of(inputs.velocity/60);
  }

   public void stop(){
     io.stop();
   }

}
