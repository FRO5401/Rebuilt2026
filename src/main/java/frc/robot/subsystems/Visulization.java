// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Utils.FuelSim;
import frc.robot.Utils.MathHelp;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class Visulization extends SubsystemBase {

  private FuelSim fuelSim;
  private Turret turret;
  private Shooter shooter;
  private Intake intake;

  private final double CAPACITY = 24;
  private double fuelStored = 8;

  private Timer shootTimer = new Timer();

  private Transform3d turretTransform = new Transform3d(TurretConstants.TURRET_TRANSFORM.getMeasureX(), TurretConstants.TURRET_TRANSFORM.getMeasureY(), Inches.of(18), Rotation3d.kZero);
  private Translation3d turretPoseTransform = new Translation3d(-0.11, 0, 0.345);
  private Translation3d intakePoseTransform = new Translation3d(0.215, 0, 0.178);


  private Supplier<Pose2d> robotPose;
  private Pose3d intakePose, turretPose;
  //    Pose3d indexer = new Pose3d(0, 0, 0.015, new Rotation3d(0, 0, Math.sin(Timer.getTimestamp())-1));


  /** Creates a new Visulization. */
  public Visulization(FuelSim fuelSim, Supplier<Pose2d> robotPose, Turret turret, Shooter shooter, Intake intake) {
    this.fuelSim = fuelSim;
    this.robotPose = robotPose;
    this.turret = turret;
    this.shooter = shooter;
    this.intake = intake;
    shootTimer.start();

    turretPose = new Pose3d(-0.11, 0, 0.345, new Rotation3d(0, 0, turret.getTurretAngle().in(Radians)));
    intakePose = new Pose3d(0.215, 0, 0.178, new Rotation3d(0, intake.getPivotPosition() - Degrees.of(70).in(Radians), 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretPose = new Pose3d(turretPoseTransform, new Rotation3d(0, 0, turret.getTurretAngle().in(Radians)));
    intakePose = new Pose3d(intakePoseTransform, new Rotation3d(0, intake.getPivotPosition() - Degrees.of(70).in(Radians), 0));
    Logger.recordOutput("Visulization/Robot Pose", robotPose.get());
    Logger.recordOutput("Visulization/Intake", intakePose);
    Logger.recordOutput("Visulization/Turret", turretPose);
    Logger.recordOutput("Visulization/Robot Components", new Pose3d[] {intakePose, turretPose});
    Logger.recordOutput("Visulization/Zeroed Components", new Pose3d[] {new Pose3d(), new Pose3d()});
    Logger.recordOutput("Current Fuel Count", fuelStored);
    if (shootTimer.advanceIfElapsed(0.25) && DriverStation.isEnabled()) {
      launchFuel();
    }
  }

  public boolean canIntake() {
        return fuelStored < CAPACITY;
    }

    public void intakeFuel() {
        fuelStored++;
    }

    public void launchFuel() {
        if (fuelStored == 0) return;
        fuelStored--;

        fuelSim.launchFuel(
                MathHelp.findFlyWheelVelocity(turret.getPoseDifference()),
                MathConstants.LAUNCH_ANGLE,
                turret.getTurretAngle(),
                turretTransform.getMeasureZ()
          );
    }
}
