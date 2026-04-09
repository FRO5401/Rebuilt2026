// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Utils.FuelSim;
import frc.robot.Utils.MathHelp;
import frc.robot.Utils.TunableNumber;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class Visualization extends SubsystemBase {

    private FuelSim fuelSim;
    private Turret turret;

    @SuppressWarnings("unused")
    private Shooter shooter;
    private Intake intake;
    private CommandSwerveDrivetrain drivetrain;

    private final double CAPACITY = 24;
    private double fuelStored = 8;

    private Timer shootTimer = new Timer();

    private Transform3d turretTransform = new Transform3d(TurretConstants.TURRET_TRANSFORM.getMeasureX(),
        TurretConstants.TURRET_TRANSFORM.getMeasureY(), Inches.of(18), Rotation3d.kZero);

    private Translation3d drivetrainTransform = new Translation3d(0, 0, -0.025);
    private Translation3d intakePoseTransform = new Translation3d(0.217, 0.0, 0.208);
    private Translation3d turretPoseTransform = new Translation3d(-0.12, 0, 0.34);
    private Translation3d hoodPoseTransform = drivetrainTransform.plus(new Translation3d(-0.09, 0, 0.445));

    private Pose3d robotPose, intakePose, turretPose, hoodPose;

    /** Creates a new Visualization. */
    public Visualization(FuelSim fuelSim, CommandSwerveDrivetrain drivetrain, Turret turret, Shooter shooter, Intake intake) {
        this.fuelSim = fuelSim;
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.shooter = shooter;
        this.intake = intake;
        shootTimer.start();

    }

    @Override
    public void periodic() {
        robotPose = new Pose3d(drivetrain.getPose());

        intakePose = new Pose3d(intakePoseTransform,
                new Rotation3d(0, Rotations.of(intake.getPivotPosition()).in(Radians) - Degrees.of(90).in(Radians), 0));

        turretPose = new Pose3d(turretPoseTransform, new Rotation3d(0, 0, turret.getTurretAngle().in(Radians)));
        hoodPose = new Pose3d(hoodPoseTransform, new Rotation3d(0, Math.sin(Timer.getTimestamp()+1), 0)).rotateAround(turretPoseTransform, turretPose.getRotation());

        Logger.recordOutput("Visualization/Robot Pose", robotPose);
        Logger.recordOutput("Visualization/Intake", intakePose);
        Logger.recordOutput("Visualization/Turret", turretPose);
        Logger.recordOutput("Visualization/Hood Pose", hoodPose);

        Logger.recordOutput("Visualization/Robot Components", new Pose3d[] {intakePose, turretPose, hoodPose});
        Logger.recordOutput("Visualization/Zeroed Components", new Pose3d[] { new Pose3d(), new Pose3d(), new Pose3d() });
        Logger.recordOutput("Visualization/Zeroed Component", new Pose3d());
        Logger.recordOutput("Visualization/HoodRotation Pitch", new Pose3d(hoodPoseTransform, new Rotation3d(0, Math.sin(Timer.getTimestamp()+1), turret.getTurretAngle().in(Radians))));
        Logger.recordOutput("Visualization/IntakeV2", new Pose3d(intakePoseTransform, new Rotation3d(0, Math.sin(Timer.getTimestamp())-1 ,0)));
        
// X: -0.113 Z:0.468
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
        if (fuelStored == 0)
            return;
        fuelStored--;

        fuelSim.launchFuel(
                MathHelp.findFlyWheelVelocity(turret.getPoseDifference()),
                MathConstants.LAUNCH_ANGLE,
                turret.getTurretAngle(),
                turretTransform.getMeasureZ());
    }
}
