// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;


import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Mass;
import frc.robot.Utils.Zones.Triangle;
import frc.robot.Utils.Zones.Rectangle.RectangleBounds;
import frc.robot.Utils.Zones.Triangle.TriangleBound;
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

                public static final Current STATOR_LIMIT = Amps.of(80);
                public static final Current SUPPLY_LIMIT = Amps.of(80);
                public static final double GEAR_RATIO = 1.45 * 3;

                public static final double KP = 70;
                public static final double KI = 0.0;
                public static final double KD = 2;

                public static final double KS = 0;
                public static final double KV = 0;

                public static final double KP_SIM = 10;
                public static final double KI_SIM = 0.0;
                public static final double KD_SIM = 0.5;

                public static final MotorOutputConfigs OUTPUT_CONFIG = new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Brake);

                public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
                                .withSensorToMechanismRatio(GEAR_RATIO);

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
                                .withKV(0)
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
                 *
                 */
                public static int ITERATIONS = 5;
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
                                .withNeutralMode(NeutralModeValue.Coast);

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

                public static final InterpolatingDoubleTreeMap TREE_MAP = new InterpolatingDoubleTreeMap();

                public static final InterpolatingDoubleTreeMap TOF_MAP = new InterpolatingDoubleTreeMap();

        public static void initializeTreeMap() {
                //TREE_MAP.put(, );
                TREE_MAP.put(2.974249165939367,55.0);
                TREE_MAP.put(2.494898698234213,50.0);

                TREE_MAP.put(2.747612404731888, 57.0);
                TREE_MAP.put(3.809172382556807, 65.0);
                TREE_MAP.put(4.323078087417264, 68.0);
                TREE_MAP.put(5.3043601660939546, 75.0);
                TREE_MAP.put(3.419481919864344, 61.0);
                TREE_MAP.put(5.598174816390024, 80.0);
                TREE_MAP.put(6.5, 105.0);

                TREE_MAP.put(1.7284305040682872, 51.0);


                // TREE_MAP.put(, );



            TOF_MAP.put(-55.37029871249218, 3.81- 2.7);
            TOF_MAP.put(-62.047688968078134, 9.17-7.76);

            TOF_MAP.put(-69.29962142071493, 2.59-1.1);
            TOF_MAP.put(-71.60450484836369, 4.23-2.57);
            TOF_MAP.put(-76.9365323598279, 4.4-2.62);
            TOF_MAP.put(-65.48803377289674, 3.05-1.57);
            TOF_MAP.put(-77.28934651220227, 11.43-9.71);


        }

        }

        public static final class IntakeConstants {
                public static final int PIVOT_MASTER_ID = 20;
                public static final int INFEED_ID = 21;

                public static final double INTAKE_SPEED = 0.5;

                public static final double PIVOT_GEAR_RATIO = 45;
                public static final double INFEED_GEAR_RATIO = 3;

                public static final double PIVOT_STATOR_LIMIT = 80;
                public static final double PIVOT_SUPPLY_LIMIT = 120;

                public static final double INFEED_SUPPLY_LIMIT = 40;
                public static final double INFEED_STATOR_LIMIT = 80;

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

                public static final double INTAKE_OUT_POSE = 0.265;

        }

        public static final class MathConstants {

                public static final Distance HUB_HEIGHT = Meters.of(1.83 - 0.345); // Height of the hub - height of the
                                                                                   // turret
                public static final Angle LAUNCH_ANGLE = Degrees.of(65);
                public static final Distance FLY_WHEEL_DIAMETER = Inches.of(3);
                public static final LinearAcceleration GRAVITY = MetersPerSecondPerSecond.of(9.81);
                public static final Mass BALL_MASS = Pound.of(0.5);
                public static final Distance BALL_DIAMETER = Inches.of(6);

                public static final double BALL_VOLUME = Math.PI * Math.pow(BALL_DIAMETER.in(Meters) / 2.0, 2);

                public static final double RHO = 1.2; // Air density kg/m^3

                public static final double FLYWHEEL_EFFICIENCY = 1; // Percentage of the flywheel's velocity that is
                                                                    // transferred
                                                                    // to the ball

                public static final double CD = 0.47; // Drag coefficient of a sphere

                public static final double AIR_RESISTANCE = RHO * CD * 0.017671458676442587 * 0.5;

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
                public static final double trackWidth = Units.inchesToMeters(22.75); // TODO: This must be tuned to
                                                                                     // specific
                                                                                     // robot
                public static final double wheelBase = Units.inchesToMeters(26.75); // TODO: This must be tuned to
                                                                                    // specific
                                                                                    // robot
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

                public enum DriveType{
                        BUMP,
                        TRENCH,
                        DEFAULT
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

                public static final Pose2d RED_HUB_TARGET = new Pose2d(11.9, 4, new Rotation2d());

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

                public static final Current STATOR_LIMIT = Amps.of(120);
                public static final Current SUPPLY_LIMIT = Amps.of(60);
                public static final double SPINDEXER_GEAR_RATIO = 3;
                public static final double KICKER_GEAR_RATIO = 1;

                public static final MotorOutputConfigs OUTPUT_CONFIG = new MotorOutputConfigs()
                                .withInverted(InvertedValue.CounterClockwise_Positive)
                                .withNeutralMode(NeutralModeValue.Coast);

                public static final MotorOutputConfigs LOADER_OUTPUT_CONFIG = new MotorOutputConfigs()
                                .withInverted(InvertedValue.Clockwise_Positive)
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
                                new Translation3d(Inches.of(-6.625).in(Meters),
                                                Inches.of(-11.5).in(Meters), Inches.of(13.25).in(Meters)),
                                new Rotation3d(0, Math.toRadians(5), Math.toRadians(180)));

                public static final Transform3d BACK_LEFT_CAMERA_POSE = new Transform3d(
                                new Translation3d(Inches.of(-7.625).in(Meters), Inches.of(13.3125).in(Meters),
                                                Inches.of(13.25).in(Meters)),
                                new Rotation3d(0, Math.toRadians(5), Math.toRadians(90)));

                public static final Transform3d FRONT_CAMERA_POSE = new Transform3d(new Translation3d(0, 0, 0),
                                new Rotation3d(0, -5, 0));

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
            public static final Distance TRAVERSAL_LENGTH = Inches.of(47);
            public static final Distance TRENCH_WIDTH = Inches.of(50.34);
            public static final Distance BUMP_FROM_WALL = Inches.of(65.65);
            public static final Distance BUMP_WIDTH = Inches.of(69.7);
            public static final Distance HUB_WIDTH = Inches.of(47);
            public static final Distance HUB_FROM_WALL = Inches.of(135.35);

            public static final RectangleBounds FIELD_ZONE = new RectangleBounds(
                    Inches.of(0), FIELD_LENGTH,
                    Inches.of(0), FIELD_WIDTH);

            public static final RectangleBounds BLUE_ZONE = new RectangleBounds(
                    Inches.of(0), ALLIANCE_FROM_WALL,
                    Inches.of(0), FIELD_WIDTH);
    
            public static final RectangleBounds RED_ZONE = BLUE_ZONE.getMirrorX();

            public static final RectangleBounds NUETRAL_BLUE_OUTPOST = new RectangleBounds(
                ALLIANCE_FROM_WALL.plus(TRAVERSAL_LENGTH), FIELD_LENGTH.minus(ALLIANCE_FROM_WALL).minus(TRAVERSAL_LENGTH), 
                Inches.of(0), FIELD_WIDTH.div(2));

            public static final RectangleBounds NUETRAL_BLUE_DEPO = NUETRAL_BLUE_OUTPOST.getMirrorY();

            public static final RectangleBounds BLUE_TRENCH_OUTPOST_ZONE = new RectangleBounds(
                ALLIANCE_FROM_WALL, ALLIANCE_FROM_WALL.plus(TRAVERSAL_LENGTH), 
                Inches.of(0), TRENCH_WIDTH);
            
            public static final RectangleBounds BLUE_TRENCH_DEPO_ZONE = BLUE_TRENCH_OUTPOST_ZONE.getMirrorY();
            public static final RectangleBounds RED_TRENCH_OUTPOST_ZONE = BLUE_TRENCH_OUTPOST_ZONE.getMirrorBounds();
            public static final RectangleBounds RED_TRENCH_DEPO_ZONE = BLUE_TRENCH_OUTPOST_ZONE.getMirrorX();

            public static final RectangleBounds BLUE_BUMP_OUTPOST_ZONE = new RectangleBounds(
                ALLIANCE_FROM_WALL, ALLIANCE_FROM_WALL.plus(TRAVERSAL_LENGTH), 
                BUMP_FROM_WALL, BUMP_FROM_WALL.plus(BUMP_WIDTH));
            
            public static final RectangleBounds BLUE_BUMP_DEPO_ZONE = BLUE_BUMP_OUTPOST_ZONE.getMirrorY();
            public static final RectangleBounds RED_BUMP_OUTPOST_ZONE = BLUE_BUMP_OUTPOST_ZONE.getMirrorBounds();
            public static final RectangleBounds RED_BUMP_DEPO_ZONE = BLUE_BUMP_OUTPOST_ZONE.getMirrorX();

            public static final TriangleBound BLUE_TURTLE_ZONE = new TriangleBound(
                new Translation2d(Inches.of(205.6), Inches.of(135.35)), 
                new Translation2d(Inches.of(205.6), Inches.of(182.1)), 
                new Translation2d(Inches.of(245.6), Inches.of((182.1+135.35)/2))
            );
            public static final TriangleBound RED_TURTLE_ZONE = BLUE_TURTLE_ZONE.getMirrorBound();

        }

}
