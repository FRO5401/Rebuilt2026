// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {

  
  private final TurretIO io;
  TurretIOInputs inputs = new TurretIOInputs();

  //Checks if the robot is real or fake, and uses the correct PID controller
  PIDController controller = RobotBase.isReal() ? new PIDController(TurretConstants.KP, TurretConstants.KI, TurretConstants.KD) 
  : new PIDController(TurretConstants.KP_SIM, TurretConstants.KI_SIM, TurretConstants.KD_SIM); 

  //target on the field
  Pose2d target;

  //I like having a supplier as the function already exists for auto and it makes the code cleaner. 
  Supplier<Pose2d> robotPose;


  //The difference of the turret post to the desired pose
  Transform2d poseDifference;


  //The current angle the turret is set too
  double currentAngle = 0;

  public Turret(TurretIO io, Supplier<Pose2d> robotPose) {
    this.io = io;
    this.robotPose = robotPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    if(robotPose != null && target != null){
      poseDifference = robotPose.get().minus(target);
      setTurretAngle(Math.atan2(poseDifference.getY(), poseDifference.getX())+poseDifference.getRotation().getRadians()+Math.PI);
    } 
    
    Logger.recordOutput("Turret Angle", currentAngle-robotPose.get().getRotation().getRadians());
    
    Logger.recordOutput("Robot Pose", robotPose.get());

    Logger.recordOutput("MotorRotation", Units.rotationsToRadians(inputs.position));
    Logger.recordOutput("sim output", controller.calculate(inputs.position, Units.radiansToRotations(currentAngle)));
    Logger.recordOutput("Inputs", inputs.voltage);

    io.applyVoltage(controller.calculate(inputs.position, Units.radiansToRotations(currentAngle)));
  }

  public void setTurretAngle(double angle){
    currentAngle = angle;
  }

  public void setTarget(Pose2d target){
    this.target = target;
  }


}
