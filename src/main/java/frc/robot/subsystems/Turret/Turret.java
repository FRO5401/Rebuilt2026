// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Utils.PhysicsSolver;
import frc.robot.Utils.ZoneGetter;
import frc.robot.Utils.TunableNumber;

public class Turret extends SubsystemBase {

  private TunableNumber kP = new TunableNumber("Turret/kp", TurretConstants.KP);
  private TunableNumber kI = new TunableNumber("Turret/ki", TurretConstants.KI);
  private TunableNumber kD = new TunableNumber("Turret/kd", TurretConstants.KD);

  private TunableNumber kS = new TunableNumber("Turret/kS", TurretConstants.KS);
  private TunableNumber kV = new TunableNumber("Turret/kV", TurretConstants.KV);

  private final TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private Translation3d[] trajectory = new Translation3d[50];
  // :)
  // Checks if the robot is real or fake, and uses the correct PID controller
  private PIDController controller = RobotBase.isReal()
      ? new PIDController(TurretConstants.KP, TurretConstants.KI, TurretConstants.KD)
      : new PIDController(TurretConstants.KP_SIM, TurretConstants.KI_SIM, TurretConstants.KD_SIM);

  private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS.get(), kV.get());

  // target on the field
  private Pose2d target;

  // I like having a supplier as the function already exists for auto and it makes
  // the code cleaner.
  private Supplier<Pose2d> robotPose;

  // The difference of the turret post to the desired pose
  private Transform2d poseDifference;

  // Robot Velocity that will affect the shot
  private Transform2d robotVelocities;

  // The current angle the turret is set too
  private double currentAngle = 0;

  // TOF used for transforming the target
  private Time tof;

  private Pose2d turretPose;

  // Chassis speeds
  private Supplier<ChassisSpeeds> fieldSpeedsSupplier;

  public Turret(TurretIO io, Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {

    this.fieldSpeedsSupplier = fieldSpeedsSupplier;

    this.io = io;
    this.robotPose = robotPose;

    controller.setTolerance(.001);
    controller.setIZone(.05);
    controller.enableContinuousInput(0, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretPose = robotPose.get().transformBy(TurretConstants.TURRET_TRANSFORM);
    io.updateInputs(inputs);

    if (getCurrentCommand() != null) {
      Logger.recordOutput("Commands/Turret Command", getCurrentCommand().getName());
    }

    if (turretPose != null && target != null) {

      poseDifference = turretPose.minus(target);
      var robotVelocities = new Transform2d(
          fieldSpeedsSupplier.get().vxMetersPerSecond * PhysicsSolver.solveTimeOfFlight(poseDifference).in(Seconds),
          fieldSpeedsSupplier.get().vyMetersPerSecond * PhysicsSolver.solveTimeOfFlight(poseDifference).in(Seconds),
          Rotation2d.kZero);

      for (int i = 0; i < TurretConstants.ITERATIONS; i++) {
        tof = PhysicsSolver.solveTimeOfFlight(poseDifference);
        poseDifference = robotPose.get().transformBy(TurretConstants.TURRET_TRANSFORM)
            .minus(target.plus(robotVelocities.inverse()));

        robotVelocities = new Transform2d(
            fieldSpeedsSupplier.get().vxMetersPerSecond * tof.in(Seconds),
            fieldSpeedsSupplier.get().vyMetersPerSecond * tof.in(Seconds),
            Rotation2d.kZero);
      }

      Logger.recordOutput("Poses/target", target.plus(robotVelocities.inverse()));

      Logger.recordOutput("Poses/DifferenceFromTarget", poseDifference);

      setTurretAngle(((Math.atan2(poseDifference.getY(), poseDifference.getX()))
          + poseDifference.getRotation().getRadians()));

    }

    Logger.recordOutput("Turret/Turret Angle", currentAngle);

    Logger.recordOutput("Turret/Robot Pose", robotPose.get());

    Logger.recordOutput("Turret Position", (inputs.position));

    Logger.recordOutput("Turret/MotorRotation",
        Units.rotationsToRadians(inputs.position) - robotPose.get().getRotation().getRadians());

    // Logger.recordOutput("Turret/Turret angle pose", new Pose3d(-0.11, 0, 0.345,
    // new Rotation3d(0, 0, Units.rotationsToRadians(inputs.position))));

    Logger.recordOutput("Turret/Turret Pose", new Pose3d(-0.11, 0, 0.345, new Rotation3d(0, 0,
        (-2 * robotPose.get().getRotation().getRadians()) + Units.rotationsToRadians(inputs.position))));

    if (!controller.atSetpoint()) {
      io.applyDutyCycle(controller.calculate(inputs.position, Units.radiansToRotations(currentAngle+Math.PI)));
    } else {
      io.applyDutyCycle(0);
    }

    Logger.recordOutput("Turret/AtSetpoint", controller.atSetpoint());

    Logger.recordOutput("Current Zone", ZoneGetter.getCurrentZone(robotPose.get()));
    Logger.recordOutput("Is Shooting Zone", ZoneGetter.isShootingZone(robotPose.get()));
    Logger.recordOutput("Current Specific Zone", ZoneGetter.getCurrentZoneSpecific(robotPose.get()));

    Logger.recordOutput("Turret/Applied",
        controller.calculate(Units.rotationsToRadians(inputs.position), -Units.radiansToRotations(currentAngle)));

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kS.hasChanged() || kV.hasChanged()) {
      setPID(kP.get(), kI.get(), kD.get(), kV.get(), kS.get());

    }

  }

  public void setTurretAngle(double angle) {

    if (Double.isNaN(angle)) {
      return;
    }
    currentAngle = angle;

  }

  public void setTarget(Pose2d target) {
    this.target = target;
  }

  public Command setSmartTarget() {
    if (target != ZoneGetter.getShootingTarget(robotPose.get())) {
      return Commands.runOnce(() -> setTarget(ZoneGetter.getShootingTarget(robotPose.get())), this);
    } else {
      return Commands.none();
    }
  }

  private Translation3d launchVel(LinearVelocity vel) {
    ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();

    double horizontalVel = Math.cos(MathConstants.LAUNCH_ANGLE.in(Radians)) * vel.in(MetersPerSecond);
    double verticalVel = Math.sin(MathConstants.LAUNCH_ANGLE.in(Radians)) * vel.in(MetersPerSecond);
    double xVel = horizontalVel
        * Math.cos(Units.rotationsToRadians(inputs.position) - robotPose.get().getRotation().getRadians());
    double yVel = horizontalVel
        * Math.sin(Units.rotationsToRadians(inputs.position) - robotPose.get().getRotation().getRadians());

    xVel += fieldSpeeds.vxMetersPerSecond;
    yVel += fieldSpeeds.vyMetersPerSecond;

    return new Translation3d(xVel, yVel, verticalVel);
  }

  public void updateFuel(LinearVelocity vel) {
    Pose3d robot = new Pose3d(robotPose.get().transformBy(TurretConstants.TURRET_TRANSFORM));
    Translation3d trajVel = launchVel(vel);
    for (int i = 0; i < trajectory.length; i++) {
      double t = i * 0.08;
      double x = trajVel.getX() * t + robot.getTranslation().getX();
      double y = trajVel.getY() * t + robot.getTranslation().getY();
      double z = trajVel.getZ() * t
          - 0.5 * 9.81 * t * t
          + robot.getTranslation().getZ();

      trajectory[i] = new Translation3d(x, y, z + 0.345);
    }
    Logger.recordOutput("Turret/Trajectory", trajectory);
  }

  public Transform2d getPoseDifference() {
    return poseDifference;
  }

  public Angle getTurretAngle() {
    return Radians.of((-2 * robotPose.get().getRotation().getRadians()) + Units.rotationsToRadians(inputs.position));
  }

  public void setPID(double P, double I, double D, double V, double S) {
    controller.setPID(P, I, D);
    feedforward.setKs(S);
    feedforward.setKv(V);
  }
}
