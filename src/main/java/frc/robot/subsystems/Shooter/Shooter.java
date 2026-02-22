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
import frc.robot.Utils.TunableNumber;
import frc.robot.subsystems.Shooter.ShooterIO.ShooterIOInputs;

public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private ShooterIOInputs inputs = new ShooterIOInputs();

  private double desiredVel = 0;

  private TunableNumber kP = new TunableNumber("Shooter/kp", ShooterConstants.CLOSED_LOOP.kP);
  private TunableNumber kI = new TunableNumber("Shooter/ki", ShooterConstants.CLOSED_LOOP.kI);
  private TunableNumber kD = new TunableNumber("Shooter/kd", ShooterConstants.CLOSED_LOOP.kD);

  public Shooter(ShooterIO io) {
    this.io = io;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    Logger.recordOutput("Shooter/Shooter velocity", getVelocity().in(RotationsPerSecond));
    Logger.recordOutput("Shooter/Shooter Desired Velocity", desiredVel);

    if(kP.hasChanged() || kI.hasChanged() || kD.hasChanged()){
      io.applyPID(kP.get(), kI.get(), kD.get());
    }

  }

  public Command setVelocity(Supplier<AngularVelocity> vel){
    return run(()->{
      io.setVelocity(vel.get().in(RotationsPerSecond), inputs);
      desiredVel = vel.get().in(RotationsPerSecond);
    });
  }

  public AngularVelocity getVelocity(){
    return RotationsPerSecond.of(inputs.velocity);
  }

   public void stop(){
     io.stop();
   }

   

}
