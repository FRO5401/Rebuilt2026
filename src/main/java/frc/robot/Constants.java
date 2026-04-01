// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import frc.robot.Utils.Zones.Triangle;
import frc.robot.Utils.Zones.Rectangle.RectangleBounds;
import frc.robot.Utils.Zones.ZoneBases.ZoneGroup;
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

    public static final class TurretConstants {

        public static final Current STATOR_LIMIT = Amps.of(60);
        public static final Current SUPPLY_LIMIT = Amps.of(60);
        public static final double GEAR_RATIO = 1.4545; // * 3;

        public static final double KP = 70;
        public static final double KI = 0;
        public static final double KD = 3.3;

        public static final double KS = 0;
        public static final double KV = 150;

        public static final double KP_SIM = 10;
        public static final double KI_SIM = 0.0;
        public static final double KD_SIM = 0.5;

        public static final CANcoder encoder = new CANcoder(27);

        public static final MotorOutputConfigs OUTPUT_CONFIG = new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
                .withSensorToMechanismRatio(GEAR_RATIO)
                .withRotorToSensorRatio(3)
                .withRemoteCANcoder(encoder)
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(STATOR_LIMIT)
                .withSupplyCurrentLimit(SUPPLY_LIMIT);

        public static final ClosedLoopGeneralConfigs CLOSED_LOOP_GENERAL_CONFIGS = new ClosedLoopGeneralConfigs()
                .withContinuousWrap(false);

        public static final Slot0Configs CLOSED_LOOP = new Slot0Configs()
                .withKP(KP)
                .withKI(KI)
                .withKD(KD)
                .withKA(0)
                .withKV(KV)
                .withKG(0);

        public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration()
                .withSlot0(CLOSED_LOOP)
                .withCurrentLimits(CURRENT_LIMITS_CONFIG)
                .withMotorOutput(OUTPUT_CONFIG)
                .withFeedback(FEEDBACK_CONFIG)
                .withClosedLoopGeneral(CLOSED_LOOP_GENERAL_CONFIGS);

        /**
         * The transform of the center of the robot to the position of the turret
         * This is used to calculate the required velocity of the shot
         */
        public static final Transform2d TURRET_TRANSFORM = new Transform2d(-0.15, 0, new Rotation2d(0));
        public static final int CAN_ID = 4;

        /**
         * Used for number of iterations of the turret angle
         */
        public static int ITERATIONS = 5;

        static {
            encoder.getConfigurator()
                    .apply(new MagnetSensorConfigs()
                            .withMagnetOffset(0)
                            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                            .withAbsoluteSensorDiscontinuityPoint(1));
        }

        public enum TurretMode {
            Static,
            Turret
        };

    }

    public static final class ShooterConstants {
        public static final int MOTOR_ID = 10;
        public static final int SHOOTER2_MOTOR_ID = 25;

        public static final double GEAR_RATIO = 1;
        public static final int MAX_VELOCITY = 5800;

        public static final double KP = 0.0415;
        public static final double KI = 0.0;
        public static final double KD = 0.001;

        public static final double KS = 0;
        public static final double KV = 0.15;
        public static final double KA = 0;

        public static final double KS_SIM = 0;
        public static final double KV_SIM = 0.0022;
        public static final double KA_SIM = 0;

        public static final double KP_SIM = 0.04093;
        public static final double KI_SIM = 0.0;
        public static final double KD_SIM = 0.0;

        public static final Current STATOR_LIMIT = Amps.of(120);
        public static final Current SUPPLY_LIMIT = Amps.of(80);

        public static final MotorOutputConfigs OUTPUT_CONFIG = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
                .withPeakForwardDutyCycle(0);

        public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
                .withSensorToMechanismRatio(GEAR_RATIO);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(STATOR_LIMIT)
                .withSupplyCurrentLimit(SUPPLY_LIMIT);

        public static final Slot0Configs CLOSED_LOOP = new Slot0Configs()
                .withKP(KP)
                .withKI(KI)
                .withKD(KD)
                .withKA(KA)
                .withKV(KV)
                .withKG(KS);

        public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration()
                .withSlot0(CLOSED_LOOP)
                .withCurrentLimits(CURRENT_LIMITS_CONFIG)
                .withMotorOutput(OUTPUT_CONFIG)
                .withFeedback(FEEDBACK_CONFIG).withMotorOutput(OUTPUT_CONFIG);

        public static final TalonFXConfiguration SHOOTER_CONFIG = new TalonFXConfiguration()
                .withSlot0(CLOSED_LOOP)
                .withCurrentLimits(CURRENT_LIMITS_CONFIG)
                .withMotorOutput(OUTPUT_CONFIG)
                .withFeedback(FEEDBACK_CONFIG).withMotorOutput(OUTPUT_CONFIG);

        public static final InterpolatingDoubleTreeMap FLYWHEEL_MAP = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

        public static void initializeTreeMap() {
            // Flywheel Velocity Interpolating Map
            FLYWHEEL_MAP.put(1.5437171376305936, 40.0);
            FLYWHEEL_MAP.put(1.9579019624329193, 44.4);
            FLYWHEEL_MAP.put(2.243642057323107, 44.9);
            FLYWHEEL_MAP.put(2.4773555790964834, 46.2);
            FLYWHEEL_MAP.put(2.580118241539039, 46.9);
            FLYWHEEL_MAP.put(2.762545007200165, 48.0);
            FLYWHEEL_MAP.put(2.9690895534584527, 48.9);
            FLYWHEEL_MAP.put(3.2598757767863358, 53.0);
            FLYWHEEL_MAP.put(3.5025031777035407, 55.1 - 2);
            FLYWHEEL_MAP.put(3.7515509451943614, 57.4 - 2);

            FLYWHEEL_MAP.put(3.931132241158132, 58.6 - 2.3);
            FLYWHEEL_MAP.put(4.234808964168099, 61.4 - 2.3);
            FLYWHEEL_MAP.put(4.486896019541133, 63.4 - 2.5);
            FLYWHEEL_MAP.put(4.737732528754779, 66.3 - 2.5);
            FLYWHEEL_MAP.put(5.039099787600017, 68.7 - 1.9);
            FLYWHEEL_MAP.put(5.232601376263012, 71.3 - 2.0);
            FLYWHEEL_MAP.put(5.546327985048606, 72.5 - 2.8);
            FLYWHEEL_MAP.put(6.006910981899345, 89.4 - 2.8);

            // Time of Flight Interpolating Map
            TOF_MAP.put(-39.51678196822748, 5.64 - 4.9);
            TOF_MAP.put(-43.8392300474685, 1.56 - 0.63);
            TOF_MAP.put(-48.564590602080635, 3.325 - 2.20);
            TOF_MAP.put(-56.61023365424546, 4.878 - 3.51);
            TOF_MAP.put(-54.45611853982277, 7.536 - 6.22);

            TOF_MAP.put(-65.01477348760973, 10.44 - 8.95);
            TOF_MAP.put(-56.603308217465546, 5.57 - 4.20);
            TOF_MAP.put(-43.4210927573636, 3.115 - 2.13);
            TOF_MAP.put(-57.324515607256586, 5.64 - 4.24);
            TOF_MAP.put(-70.4731497869255, 7.54 - 5.9);
            TOF_MAP.put(-83.5442041114383, 13.53 - 11.65);

        }

    }

    public static final class IntakeConstants {
        public static final int PIVOT_MASTER_ID = 20;
        public static final int PIVOT_FOLLOWER_ID = 26;

        public static final int INFEED_ID = 21;

        public static final double INTAKE_SPEED = 0.5;

        public static final double PIVOT_GEAR_RATIO = 45;
        public static final double INFEED_GEAR_RATIO = 9;

        public static final double PIVOT_STATOR_LIMIT = 60;
        public static final double PIVOT_SUPPLY_LIMIT = 80;

        public static final double INFEED_SUPPLY_LIMIT = 40;
        public static final double INFEED_STATOR_LIMIT = 60;

        public static final boolean PIVOT_MASTER_INVERT = false;
        public static final boolean PIVOT_FOLLOWER_INVERT = true;

        public static final boolean INFEED_INVERT = false;

        public static final double kp = 50;
        public static final double ki = 0;
        public static final double kd = 5;
        public static final double ks = 0;
        public static final double kv = 0;

        public static final double sim_kp = 1;
        public static final double sim_ki = 1;
        public static final double sim_kd = 1;
        public static final double sim_ks = 1;

        public static final Slot0Configs CLOSED_LOOP = new Slot0Configs()
                .withKP(kp)
                .withKI(ki)
                .withKD(kd)
                .withKS(ks)
                .withKV(kv);

        public static final double INTAKE_OUT_POSE = 0.269;

    }

    public static final class MathConstants {
        // Height of the hub - height of the turret
        public static final Distance HUB_HEIGHT = Meters.of(1.83 - 0.345); 
        public static final Angle LAUNCH_ANGLE = Degrees.of(65);
        public static final Distance FLY_WHEEL_DIAMETER = Inches.of(3);
        public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);
        public static final Mass BALL_MASS = Pound.of(0.5);
        public static final Distance BALL_DIAMETER = Inches.of(6);

        public static final double BALL_VOLUME = Math.PI * Math.pow(BALL_DIAMETER.in(Meters) / 2.0, 2);

        // Air density kg/m^3
        public static final double RHO = 1.2; 

        // Percentage of the flywheel's velocity that is transferred to the ball
        public static final double FLYWHEEL_EFFICIENCY = 1; 

        // Drag coefficient of a sphere
        public static final double CD = 0.47; 

        public static final double AIR_RESISTANCE = RHO * CD * 0.017671458676442587 * 0.5;

    }

    public class AutoConstants {

        public static final double kMaxSpeedMetersPerSecond = 3.5;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1.687;
        public static final double kPYController = 1.687;

        public static final double kPThetaController = 1.2;
        public static final double kDThetaController = .8;

        /* Constraint for kpx motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, kDThetaController,
                Constants.AutoConstants.kThetaControllerConstraints);

    };

    public static final class Swerve {
        public static final double trackWidth = Units.inchesToMeters(21);
        public static final double wheelBase = Units.inchesToMeters(22.25);
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

        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts
                                                                                                  // desired
                                                                                                  // top speed
        public static final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

        public enum DriveType {
            BUMP,
            TRENCH,
            FIELD_CENTRIC,
            ROBOT_CENTRIC,
            BRAKE
        }
    }

    public static final class FieldConstants {

        /*
         * Field dimensions:
         * Width: 317.6 Inches
         * Inches: 651.2 Inches
         * Zones: 158.6 Inches from each wall
         */

        public static final double HALF_WAY_LINE = Units.inchesToMeters(317.7 / 2.0);
        public static final Rectangle2d fieldZone = new Rectangle2d(
                new Translation2d(),
                new Translation2d(Inches.of(651.2), Inches.of(317.7)));
        public static final Rectangle2d blueZone = new Rectangle2d(
                new Translation2d(),
                new Translation2d(Inches.of(158.6), Inches.of(317.7)));
        public static final Rectangle2d blueTrench = new Rectangle2d(
                new Translation2d(Inches.of(158.6), Inches.of(0)),
                new Translation2d(Inches.of(205.6), Inches.of(317.7)) // 65.65 - 50.34 = 15.31
        );
        public static final Rectangle2d blueTrenchBlock = new Rectangle2d(
                new Translation2d(Inches.of(158.6), Inches.of(50.34)),
                new Translation2d(Inches.of(205.6), Inches.of(267.36)));
        public static final Rectangle2d blueBump = new Rectangle2d(
                new Translation2d(Inches.of(158.6), Inches.of(65.65)),
                new Translation2d(Inches.of(205.6), Inches.of(251.05)));
        public static final Rectangle2d blueHub = new Rectangle2d(
                new Translation2d(Inches.of(158.6), Inches.of(135.35)),
                new Translation2d(Inches.of(205.6), Inches.of(182.1)));
        public static final Rectangle2d redZone = new Rectangle2d(
                new Translation2d(Inches.of(492.6), Inches.of(0)),
                new Translation2d(Inches.of(651.2), Inches.of(317.7)));
        public static final Rectangle2d redTrench = new Rectangle2d(
                new Translation2d(Inches.of(492.6), Inches.of(0)),
                new Translation2d(Inches.of(445.6), Inches.of(317.7)));
        public static final Rectangle2d redTrenchBlock = new Rectangle2d(
                new Translation2d(Inches.of(492.6), Inches.of(50.34)),
                new Translation2d(Inches.of(445.6), Inches.of(267.36)));
        public static final Rectangle2d redBump = new Rectangle2d(
                new Translation2d(Inches.of(492.6), Inches.of(65.65)),
                new Translation2d(Inches.of(445.6), Inches.of(251.05)));
        public static final Rectangle2d redHub = new Rectangle2d(
                new Translation2d(Inches.of(492.6), Inches.of(135.35)),
                new Translation2d(Inches.of(445.6), Inches.of(182.1)));

        public static final Pose2d BLUE_HUB_TARGET = new Pose2d(4.5, 4, new Rotation2d());

        public static final Pose2d RED_HUB_TARGET = new Pose2d(11.75, 4, new Rotation2d());

        public static final Pose2d BLUE_RIGHT_PASSING_TARGET = new Pose2d(1, 2, new Rotation2d());

        public static final Pose2d BLUE_LEFT_PASSING_TARGET = new Pose2d(1, 7, new Rotation2d());

        public static final Pose2d RED_LEFT_PASSING_TARGET = new Pose2d(15, 7, new Rotation2d());

        public static final Pose2d RED_RIGHT_PASSING_TARGET = new Pose2d(15, 2, new Rotation2d());

        public enum CurrentZone {
            RED,
            RED_TRENCH,
            RED_BUMP,
            BLUE,
            BLUE_TRENCH,
            BLUE_BUMP,
            NUETRAL_LEFT,
            NUETRAL_RIGHT,
            PHASING,
            OUTSIDE_BOUNDS
        }

    }

    public static final class IndexerConstants {
        public static final int SPINDEXER_ID = 18;
        public static final int KICKER_ID = 12;

        public static final Current STATOR_LIMIT = Amps.of(80);
        public static final Current SUPPLY_LIMIT = Amps.of(60);
        public static final double SPINDEXER_GEAR_RATIO = 3;
        public static final double KICKER_GEAR_RATIO = 1;

        public static final MotorOutputConfigs OUTPUT_CONFIG = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

        public static final MotorOutputConfigs LOADER_OUTPUT_CONFIG = new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);

        public static final FeedbackConfigs SPINDEXER_FEEDBACK_CONFIG = new FeedbackConfigs()
                .withSensorToMechanismRatio(SPINDEXER_GEAR_RATIO);

        public static final FeedbackConfigs KICKER_FEEDBACK_CONFIG = new FeedbackConfigs()
                .withSensorToMechanismRatio(KICKER_GEAR_RATIO);

        public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(STATOR_LIMIT)
                .withSupplyCurrentLimit(SUPPLY_LIMIT);

        public static final Slot0Configs SPINDEXER_CLOSED_LOOP = new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
                .withKA(0)
                .withKV(0)
                .withKG(0);

        public static final Slot0Configs KICKER_CLOSED_LOOP = new Slot0Configs()
                .withKP(0)
                .withKI(0)
                .withKD(0)
                .withKA(0)
                .withKV(0)
                .withKG(0);

        public static final TalonFXConfiguration SPINDEXER_CONFIG = new TalonFXConfiguration()
                .withSlot0(SPINDEXER_CLOSED_LOOP)
                .withCurrentLimits(CURRENT_LIMITS_CONFIG)
                .withMotorOutput(OUTPUT_CONFIG)
                .withFeedback(SPINDEXER_FEEDBACK_CONFIG);

        public static final TalonFXConfiguration KICKER_CONFIG = new TalonFXConfiguration()
                .withSlot0(KICKER_CLOSED_LOOP)
                .withCurrentLimits(CURRENT_LIMITS_CONFIG)
                .withMotorOutput(LOADER_OUTPUT_CONFIG)
                .withFeedback(KICKER_FEEDBACK_CONFIG);
    }

    public static final class VisionConstants {
        public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
                .loadField(AprilTagFields.k2026RebuiltWelded);
        public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

        public static final Transform3d BACK_RIGHT_CAMERA_POSE = new Transform3d(
                new Translation3d(Inches.of(-12.6875).in(Meters),
                        Inches.of(-12.5625).in(Meters), Inches.of(16.0625).in(Meters)),
                new Rotation3d(Math.toRadians(90), Math.toRadians(5), Math.toRadians(180)));

        public static final Transform3d BACK_LEFT_CAMERA_POSE = new Transform3d(
                new Translation3d(Inches.of(-7.625).in(Meters), Inches.of(13.3125).in(Meters),
                        Inches.of(13.25).in(Meters)),
                new Rotation3d(0, Math.toRadians(5), Math.toRadians(90)));

        public static final Transform3d FRONT_CAMERA_POSE = new Transform3d(new Translation3d(0, 0, 0),
                new Rotation3d(0, -5, 0));

        public static final Matrix<N3, N1> SINGLE_TAG_STDDEV = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> MULTI_TAG_STDDEV = VecBuilder.fill(0.5, 0.5, 1);

    }

    public static final class RobotDimensionConstants {
        public static final Distance WIDTH_WBUMPERS = Inches.of(34.56);
        public static final Distance LENGTH_WBUMPERS = Inches.of(34.560082);
        public static final Distance HEIGHT_OF_BUMPERS = Inches.of(5.858);
        public static final Distance INTAKE_LENGTH = Inches.of(8.345);

        public static final Distance INTAKE_XMIN = LENGTH_WBUMPERS.div(2);
        public static final Distance INTAKE_XMAX = LENGTH_WBUMPERS.div(2);
        public static final Distance INTAKE_YMIN = WIDTH_WBUMPERS.div(2).times(-1);
        public static final Distance INTAKE_YMAX = WIDTH_WBUMPERS.div(2);

        public static final Transform2d FRONT_LEFT_CORNER = new Transform2d(INTAKE_XMAX, INTAKE_YMAX,
                Rotation2d.kZero);
        public static final Transform2d FRONT_RIGHT_CORNER = new Transform2d(INTAKE_XMAX, INTAKE_YMAX.times(-1),
                Rotation2d.kZero);
        public static final Transform2d BACK_LEFT_CORNER = new Transform2d(INTAKE_XMAX.times(-1), INTAKE_YMAX,
                Rotation2d.kZero);
        public static final Transform2d BACK_RIGHT_CORNER = new Transform2d(INTAKE_XMAX.times(-1),
                INTAKE_YMAX.times(-1), Rotation2d.kZero);
        public static final Transform2d BACK_SIDE = new Transform2d(INTAKE_XMAX.times(-1), Inches.of(0),
                Rotation2d.kZero);

    }

    public static final class FieldZones {
        /*
         * Field dimensions:
         * Width: 317.6 Inches
         * Length: 651.2 Inches
         * Zones: 158.6 Inches from each wall
         */
        public static final Distance FIELD_LENGTH = Inches.of(651.2);
        public static final Distance FIELD_WIDTH = Inches.of(317.6);
        public static final Distance ALLIANCE_FROM_WALL = Inches.of(158.6);

        public static final RectangleBounds FIELD_ZONE = new RectangleBounds(Inches.of(0), FIELD_LENGTH,
                Inches.of(0),
                FIELD_WIDTH);
        public static final ZoneGroup group = new ZoneGroup(FIELD_ZONE, Triangle.turtly);

    }

}
