// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.RobotDimensionConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.Utils.MathHelp;
import frc.robot.Utils.simulation.FuelSim;

public class Visulization extends SubsystemBase {

    private FuelSim fuelSim;
    private Turret turret;

    @SuppressWarnings("unused")
    private Shooter shooter;
    private Intake intake;

    private final int CAPACITY = 40;
    private static int fuelStored = 40;

    private Timer shootTimer = new Timer();

    private Transform3d turretTransform = new Transform3d(TurretConstants.TURRET_TRANSFORM.getMeasureX(),
            TurretConstants.TURRET_TRANSFORM.getMeasureY(), Inches.of(18), Rotation3d.kZero);

    private Translation3d turretPoseTransform = new Translation3d(-0.12, 0, 0.315);
    private Translation3d intakePoseTransform = new Translation3d(0.215, 0, 0.178);

  private Supplier<Pose3d> robotPose;
  private Pose3d intakePose, turretPose;
  private CommandXboxController operator;
  private double BPS = 6;
  private double ballLength = 4;
  private double ballWidth = 5;

    private static double hopperSizeY = RobotDimensionConstants.LENGTH_WBUMPERS.minus(Inches.of(7)).in(Meters);
    private static double hopperSizeX = RobotDimensionConstants.WIDTH_WBUMPERS.in(Meters);
    private static double hopperSizeZ = Units.inchesToMeters(21);
    private static double hopperCenterZ = Units.inchesToMeters(13.5);
  //    Pose3d indexer = new Pose3d(0, 0, 0.015, new Rotation3d(0, 0, Math.sin(Timer.getTimestamp())-1));

  /** Creates a new Visulization. */
  public Visulization(FuelSim fuelSim, Supplier<Pose3d> robotPose, Turret turret, Shooter shooter, Intake intake, CommandXboxController operator) {
    this.fuelSim = fuelSim;
    this.robotPose = robotPose;
    this.turret = turret;
    this.shooter = shooter;
    this.intake = intake;
    this.operator = operator;
    shootTimer.start();

  }

    @Override
    public void periodic() {
        turretPose = new Pose3d(turretPoseTransform, new Rotation3d(0, 0, turret.getTurretAngle().minus(Rotations.of(0.25)).in(Radians)));
        intakePose = new Pose3d(intakePoseTransform,
                new Rotation3d(0, Rotations.of(intake.getPivotPosition()).in(Radians) - Degrees.of(95).in(Radians), 0));
    
        Logger.recordOutput("Visulization/Robot Pose", robotPose.get());
        Logger.recordOutput("Visulization/Intake", intakePose);
        Logger.recordOutput("Visulization/Turret", turretPose);
        Logger.recordOutput("Visulization/Robot Components", new Pose3d[] { intakePose, turretPose });
        Logger.recordOutput("Visulization/Zeroed Components", new Pose3d[] { new Pose3d(), new Pose3d() });
        Logger.recordOutput("Current Fuel Count", fuelStored);
        Logger.recordOutput("Fuel in Hopper", getHopperFuelFieldPositions(robotPose.get()));

        if (shootTimer.advanceIfElapsed(1/BPS) && DriverStation.isEnabled() && (operator.rightTrigger().getAsBoolean() || DriverStation.isAutonomousEnabled())) {
            launchFuel();
        }

        Logger.recordOutput("Visulization/Can Intake", canIntake());

        
    }

    public boolean canIntake() {
        return fuelStored < CAPACITY && intake.isIntakeDeployed() && operator.leftTrigger().getAsBoolean(); // && intake.isIntakeDeployed() && intaking
    }

    public void intakeFuel() {
        fuelStored++;
    }

    public void launchFuel() {
        if (fuelStored == 0)
            return;
        fuelStored--;

        fuelSim.launchFuel(
                MathHelp.findFlyWheelVelocity(turret.getPoseDifference()),
                MathConstants.LAUNCH_ANGLE,
                turret.getTurretAngle(),
                turretTransform.getMeasureZ());
    }

    public Translation3d[] getHopperFuelFieldPositions(Pose3d robotPose) {
        if(robotPose == null) return new Translation3d[]{};
        Translation3d[] positions = new Translation3d[fuelStored];
        
        for(int i = 0; i < fuelStored; i++) {
            double x = ((i % 4) * (hopperSizeX / 5) - hopperSizeX / 3 + hopperSizeX / 4);
            double y = (((i / 5) % 5) * (hopperSizeY / 5) - hopperSizeY / 3 + hopperSizeY / 12)-0.03;
            double z = (i / 16) * (hopperSizeZ / 5) + hopperCenterZ - hopperSizeZ / 2 + hopperSizeZ / 12;
            positions[i] = robotPose.plus(
                new Transform3d(new Translation3d(-x, y, z), Rotation3d.kZero)
            ).plus(new Transform3d(RobotDimensionConstants.HOPPER_SHIFT.in(Meters), 0, 0, Rotation3d.kZero)).getTranslation();
        }

        return positions;
    }


}
