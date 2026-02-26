// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.RobotDimensionConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Utils.MathHelp;
import frc.robot.Utils.ZoneGetter;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Turret.Turret;

public class Visulization extends SubsystemBase {

  private Turret turret;

  @SuppressWarnings("unused")
  private Shooter shooter;
  private Intake intake;
  private CommandSwerveDrivetrain drivetrain;
  private IntakeSimulation intakeSim;


  private final double CAPACITY = 24;
  private double fuelStored = 8;

  private Transform3d turretTransform = new Transform3d(TurretConstants.TURRET_TRANSFORM.getMeasureX(), TurretConstants.TURRET_TRANSFORM.getMeasureY(), Inches.of(18), Rotation3d.kZero);
  private Translation3d turretPoseTransform = new Translation3d(-0.11, 0, 0.345);
  private Translation3d intakePoseTransform = new Translation3d(0.215, 0, 0.178);


  private Pose3d intakePose, turretPose;
  //    Pose3d indexer = new Pose3d(0, 0, 0.015, new Rotation3d(0, 0, Math.sin(Timer.getTimestamp())-1));


  public Visulization(CommandSwerveDrivetrain drivetrain, Turret turret, Shooter shooter, Intake intake) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.shooter = shooter;
    this.intake = intake;

    this.intakeSim = IntakeSimulation.OverTheBumperIntake(
      "Fuel", 
      drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive, 
      RobotDimensionConstants.WIDTH_WBUMPERS.minus(Inches.of(3)), 
      RobotDimensionConstants.INTAKE_LENGTH, 
      IntakeSide.FRONT, 
      35
    ); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    turretPose = new Pose3d(turretPoseTransform, new Rotation3d(0, 0, turret.getTurretAngle().in(Radians)));
    intakePose = new Pose3d(intakePoseTransform, new Rotation3d(0, intake.getPivotPosition() - Degrees.of(70).in(Radians), 0));
    Logger.recordOutput("Visulization/Robot Pose", drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
    Logger.recordOutput("Visulization/Intake", intakePose);
    Logger.recordOutput("Visulization/Turret", turretPose);
    Logger.recordOutput("Visulization/Robot Components", new Pose3d[] {intakePose, turretPose});
    Logger.recordOutput("Visulization/Zeroed Components", new Pose3d[] {new Pose3d(), new Pose3d()});
    Logger.recordOutput("Visulization/Current Fuel Count", intakeSim.getGamePiecesAmount());
    Logger.recordOutput("Visulization/is Intaking", intake.isIntakeRunning());
    Logger.recordOutput("Visulization/Intake Velocity", intake.getIntakeVelocity());

    if(intake.isIntakeRunning()) intakeSim.startIntake();
    else intakeSim.stopIntake();

    if(ZoneGetter.isShootingZone(drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose()))
      launchFuel();
  }
  public boolean isFuelInsideIntake(){
    return intakeSim.getGamePiecesAmount() != 0;
  }
  public void launchFuel() {
    if (intakeSim.obtainGamePieceFromIntake()){
      gamePieceProjectile();
    }         
  }
  public void gamePieceProjectile(){
    SimulatedArena.getInstance()
      .addGamePieceProjectile(
        new RebuiltFuelOnFly(
          drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getTranslation(),
          new Translation2d(-0.15, 0), 
          drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(), 
          new Rotation2d(turret.getTrueTurretAngle()), 
          Inches.of(19), 
          MathHelp.findFlyWheelVelocity(turret.getPoseDifference()), 
          MathConstants.LAUNCH_ANGLE
        ).withProjectileTrajectoryDisplayCallBack(
            (poses) -> Logger.recordOutput("Visulization/successfulShotsTrajectory", poses.toArray(Pose3d[]::new)),
            (poses) -> Logger.recordOutput("Visulization/missedShotsTrajectory", poses.toArray(Pose3d[]::new))));
      
  }

}
