// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
  public static boolean isTuningMode = true;
  public static final class TurretConstants {

    public static final Current STATOR_LIMIT = Amps.of(120);
    public static final Current SUPPLY_LIMIT = Amps.of(20);
    public static final double GEAR_RATIO = 25;

    public static final double KP = 0.5;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double KP_SIM = 50;
    public static final double KI_SIM = 0.0;
    public static final double KD_SIM = 0;

    public static final MotorOutputConfigs OUTPUT_CONFIG = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

    public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
        .withSensorToMechanismRatio(GEAR_RATIO);

    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
        .withStatorCurrentLimit(STATOR_LIMIT)
        .withSupplyCurrentLimit(SUPPLY_LIMIT);

    public static final Slot0Configs CLOSED_LOOP = new Slot0Configs()
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withKA(0)
        .withKV(0)
        .withKG(0);

    public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration()
        .withSlot0(CLOSED_LOOP)
        .withCurrentLimits(CURRENT_LIMITS_CONFIG)
        .withMotorOutput(OUTPUT_CONFIG)
        .withFeedback(FEEDBACK_CONFIG);

    /**
     * The transform of the center of the robot to the position of the turret
     * This is used to calculate the required velocity of the shot
     */
    public static final Transform2d TURRET_TRANSFORM = new Transform2d(-0.15, 0, new Rotation2d(0));

    /**
     * Used for number of iterations of the turret angle
     *
     */
    public static int ITERATIONS = 20;
  }

  public static final class ShooterConstants {
    public static final int MOTOR_ID = 0;
    public static final double GEAR_RATIO = 1;
    public static final int MAX_VELOCITY = 5800;

    public static final double KP = 0.5;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static double KS_SIM = 0;
    public static double KV_SIM = 0.0022;
    public static double KA_SIM = 0;
    
    public static double KP_SIM = 0.04093;
    public static double KI_SIM = 0.0;
    public static double KD_SIM = 0.0;

  }

  public static final class MathConstants {

    public static final Distance HUB_HEIGHT = Meters.of(1.83-0.345); // Height of the hub - height of the turret
    public static final Angle LAUNCH_ANGLE = Degrees.of(65);
    public static final Distance FLY_WHEEL_DIAMETER = Inches.of(4);

  }

  public class AutoConstants {

    public static final double kMaxSpeedMetersPerSecond = 3.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.687;
    public static final double kPYController = 1.687;

    public static final double kPThetaController = 1.687;
    public static final double kDThetaController = kPThetaController / 2;

    /* Constraint for kpx motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    public static final ProfiledPIDController thetaController = new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, kDThetaController,
        Constants.AutoConstants.kThetaControllerConstraints);

  };

  public static final class Swerve {
    public static final double trackWidth = Units.inchesToMeters(22.75); // TODO: This must be tuned to specific robot
    public static final double wheelBase = Units.inchesToMeters(26.75); // TODO: This must be tuned to specific robot
    public static final double wheelCircumference = 4 * Math.PI;

    /*
     * Swerve Kinematics
     * No need to ever change this unless you are not doing a traditional
     * rectangular/square 4 module swerve
     */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                              // top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
  }

  public static final class FieldConstants {
    /*
     * Field dimensions:
     * Width: 317.6 Inches
     * Inches: 651.2 Inches
     * Zones: 158.6 Inches from each wall
     */
    public static final Rectangle2d fieldZone = new Rectangle2d(
      new Translation2d(), 
      new Translation2d(Inches.of(651.2), Inches.of(317.7))
    );
    public static final Rectangle2d blueZone = new Rectangle2d(
      new Translation2d(), 
      new Translation2d(Inches.of(158.6), Inches.of(317.7))
    );
    public static final Rectangle2d blueTrench = new Rectangle2d(
      new Translation2d(Inches.of(158.6), Inches.of(0)), 
      new Translation2d(Inches.of(205.6), Inches.of(317.7)) //  65.65 - 50.34 = 15.31 
    );
    public static final Rectangle2d blueTrenchBlock = new Rectangle2d(
      new Translation2d(Inches.of(158.6), Inches.of(50.34)), 
      new Translation2d(Inches.of(205.6), Inches.of(267.36))
    ); 
    public static final Rectangle2d blueBump = new Rectangle2d(
      new Translation2d(Inches.of(158.6), Inches.of(65.65)), 
      new Translation2d(Inches.of(205.6), Inches.of(251.05))
    );
    public static final Rectangle2d blueHub = new Rectangle2d(
      new Translation2d(Inches.of(158.6), Inches.of(135.35)), 
      new Translation2d(Inches.of(205.6), Inches.of(182.1)) 
    );
    public static final Rectangle2d redZone = new Rectangle2d(
      new Translation2d(Inches.of(492.6), Inches.of(0)), 
      new Translation2d(Inches.of(651.2), Inches.of(317.7))
    );
    public static final Rectangle2d redTrench = new Rectangle2d(
      new Translation2d(Inches.of(492.6), Inches.of(0)), 
      new Translation2d(Inches.of(445.6), Inches.of(317.7)) 
    );
    public static final Rectangle2d redTrenchBlock = new Rectangle2d(
      new Translation2d(Inches.of(492.6), Inches.of(50.34)), 
      new Translation2d(Inches.of(445.6), Inches.of(267.36)) 
    );
    public static final Rectangle2d redBump = new Rectangle2d(
      new Translation2d(Inches.of(492.6), Inches.of(65.65)), 
      new Translation2d(Inches.of(445.6), Inches.of(251.05)) 
    );
    public static final Rectangle2d redHub = new Rectangle2d(
      new Translation2d(Inches.of(492.6), Inches.of(135.35)), 
      new Translation2d(Inches.of(445.6), Inches.of(182.1)) 
    );

    public static final Pose2d BLUE_HUB_TARGET = new Pose2d(4.5, 4, new Rotation2d());

    public static final Pose2d RED_HUB_TARGET = new Pose2d(11.9, 4, new Rotation2d());

    public static final Pose2d BLUE_PASSING_TARGET = new Pose2d(2, 2, new Rotation2d());

    public static final Pose2d RED_PASSING_TARGET = new Pose2d(14, 7, new Rotation2d());
    

    public enum CurrentZone {
      RED,
      RED_TRENCH,
      RED_BUMP,
      BLUE,
      BLUE_TRENCH,
      BLUE_BUMP,
      NUETRAL,
      PHASING,
      OUTSIDE_BOUNDS
    }


  }
}
