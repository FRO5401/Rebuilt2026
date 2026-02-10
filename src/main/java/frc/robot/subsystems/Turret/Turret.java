// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Utils.MathHelp;
import frc.robot.subsystems.Turret.TurretIO.TurretIOInputs;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  TurretIOInputs inputs = new TurretIOInputs();
  private Translation3d[] trajectory = new Translation3d[50];

  // Checks if the robot is real or fake, and uses the correct PID controller
  PIDController controller = RobotBase.isReal()
      ? new PIDController(TurretConstants.KP, TurretConstants.KI, TurretConstants.KD)
      : new PIDController(TurretConstants.KP_SIM, TurretConstants.KI_SIM, TurretConstants.KD_SIM);

  // target on the field
  Pose2d target;

  // I like having a supplier as the function already exists for auto and it makes
  // the code cleaner.
  Supplier<Pose2d> robotPose;

  // The difference of the turret post to the desired pose
  Transform2d poseDifference;

  // Robot Velocity that will affect the shot
  Transform2d robotVelocities;

  // The current angle the turret is set too
  double currentAngle = 0;

  // TOF used for transforming the target
  Time tof;

  // Chassis speeds
  Supplier<ChassisSpeeds> fieldSpeedsSupplier;

  public Turret(TurretIO io, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    controller.enableContinuousInput(0, 1);
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;

    this.io = io;
    this.robotPose = robotPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);

    if (robotPose != null && target != null) {

      poseDifference = robotPose.get().minus(target);
      var robotVelocities = new Transform2d(
          fieldSpeedsSupplier.get().vxMetersPerSecond * MathHelp.findTOF(poseDifference).in(Seconds),
          fieldSpeedsSupplier.get().vyMetersPerSecond * MathHelp.findTOF(poseDifference).in(Seconds), Rotation2d.kZero)
          .times(1.1);

      for (int i = 0; i < TurretConstants.ITERATIONS; i++) {
        tof = MathHelp.findTOF(poseDifference);
        poseDifference = robotPose.get().minus(target.plus(robotVelocities.inverse()));

        robotVelocities = new Transform2d(
            fieldSpeedsSupplier.get().vxMetersPerSecond * tof.in(Seconds),
            fieldSpeedsSupplier.get().vyMetersPerSecond * tof.in(Seconds),
            Rotation2d.kZero);
      }

      Logger.recordOutput("target", target.plus(robotVelocities.inverse()));

      Logger.recordOutput("Poses/Difference", poseDifference);

      setTurretAngle((Math.atan2(poseDifference.getY(), poseDifference.getX())
          + poseDifference.getRotation().getRadians() + Math.PI));

    }

    if (robotPose != null && target != null && fieldSpeedsSupplier.get() != null) {
      updateFuel();
    }

    Logger.recordOutput("Turret Angle", currentAngle - robotPose.get().getRotation().getRadians());

    Logger.recordOutput("Robot Pose", robotPose.get());

    Logger.recordOutput("MotorRotation",
        Units.rotationsToRadians(inputs.position) - robotPose.get().getRotation().getRotations());

    io.applyVoltage(controller.calculate(inputs.position, Units.radiansToRotations(currentAngle)));

    Logger.recordOutput("Current Zone", getCurrentZone());
    Logger.recordOutput("Is Shooting Zone", isShootingZone());

  }

  public void setTurretAngle(double angle) {
    currentAngle = angle;
  }

  public void setTarget(Pose2d target) {
    this.target = target;
  }

  private Translation3d launchVel(LinearVelocity vel) {
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    double horizontalVel = Math.cos(MathConstants.LAUNCH_ANGLE.in(Radians)) * vel.in(MetersPerSecond);
    double verticalVel = Math.sin(MathConstants.LAUNCH_ANGLE.in(Radians)) * vel.in(MetersPerSecond);
    double xVel = horizontalVel * Math.cos(currentAngle - robotPose.get().getRotation().getRadians());
    double yVel = horizontalVel * Math.sin(currentAngle - robotPose.get().getRotation().getRadians());

    xVel += fieldSpeeds.vxMetersPerSecond * tof.in(Seconds);
    yVel += fieldSpeeds.vyMetersPerSecond * tof.in(Seconds);

    return new Translation3d(xVel, yVel, verticalVel);
  }

  public void updateFuel() {
    Pose3d robot = new Pose3d(robotPose.get());
    Translation3d trajVel = launchVel(MathHelp.findFlyWheelVelocity(poseDifference));
    for (int i = 0; i < trajectory.length; i++) {
      double t = i * 0.04;
      double x = trajVel.getX() * t + robot.getTranslation().getX();
      double y = trajVel.getY() * t + robot.getTranslation().getY();
      double z = trajVel.getZ() * t
          - 0.5 * 9.81 * t * t
          + robot.getTranslation().getZ();

      trajectory[i] = new Translation3d(x, y, z);
    }
    Logger.recordOutput("Turret/Trajectory", trajectory);
  }

  Pose2d fieldLimits = new Pose2d(Inches.of(651.2), Inches.of(317.7), Rotation2d.kZero);

  Pose2d blueZoneStart = new Pose2d(Inches.of(0), Inches.of(0), Rotation2d.kZero);
  Pose2d blueZoneEnd = new Pose2d(Inches.of(158.6), Inches.of(317.7), Rotation2d.kZero);

  Pose2d redZoneStart = new Pose2d(Inches.of(651.2), Inches.of(0), Rotation2d.kZero);
  Pose2d redZoneEnd = new Pose2d(Inches.of(492.6), Inches.of(317.7), Rotation2d.kZero);

  public enum CurrentZone{
    RED,
    BLUE,
    NUETRAL,
    OUTSIDE_BOUNDS
  }

  public CurrentZone getCurrentZone(){
    Pose2d currentPose = robotPose.get();
    if(blueZoneStart.getX()>currentPose.getX() 
      || fieldLimits.getX()<currentPose.getX() 
      || blueZoneStart.getY()>currentPose.getY()
      || fieldLimits.getY()<currentPose.getY()
      ){
        return CurrentZone.OUTSIDE_BOUNDS;
      }
    if(redZoneStart.getX() >= currentPose.getX() && currentPose.getX() >= redZoneEnd.getX()){
      return CurrentZone.RED;

    } else if(blueZoneStart.getX() <= currentPose.getX() && currentPose.getX() <= blueZoneEnd.getX()){
      return CurrentZone.BLUE;

    } else {
      return CurrentZone.NUETRAL;
    }
  }

  public boolean isShootingZone(){
    if(DriverStation.getAlliance().get().equals(Alliance.Blue)){
      return getCurrentZone().equals(CurrentZone.BLUE);
    } else {
      return getCurrentZone().equals(CurrentZone.RED);
    }

  }

  

}
