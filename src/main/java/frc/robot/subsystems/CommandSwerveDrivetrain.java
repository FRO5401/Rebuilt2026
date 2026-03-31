package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.ironmaple.simulation.drivesims.COTS;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.utils.simulation.MapleSimSwerveDrivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private final PhotonCamera backRightCamera;
    private final PhotonCamera backLeftCamera;
    private final PhotonCamera frontCamera;

    public PhotonPoseEstimator backRightPoseEstimator;
    public PhotonPoseEstimator frontPoseEstimator;
    public PhotonPoseEstimator backLeftPoseEstimator;

    public List<PhotonPipelineResult> backRightResults;
    public List<PhotonPipelineResult> backLeftResults;
    public List<PhotonPipelineResult> frontResults;

    public Long firstSampleTime = System.nanoTime();
    public Long secondSampleTime = System.nanoTime();

    public LinearFilter chasisFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    public Pose2d lastPose = new Pose2d();

    private final PIDController xController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(10.0, 0.0, 0.0);
    private final PIDController headingController = new PIDController(5, 0.0, 0.0);

    Pigeon2 pigeon2;

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private final SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(.1)
            .withRotationalDeadband(.1)
            .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.FieldCentric robotFieldCentric = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withDeadband(.1)
            .withRotationalDeadband(.1)
            .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.ApplyFieldSpeeds m_applyFieldSpeeds = new SwerveRequest.ApplyFieldSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    /*
     * SysId routine for characterizing translation. This is used to find PID gains
     * for the drive motors.
     */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> setControl(m_translationCharacterization.withVolts(output)),
                    null,
                    this));

    /*
     * SysId routine for characterizing steer. This is used to find PID gains for
     * the steer motors.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                    null, // Use default ramp rate (1 V/s)
                    Volts.of(7), // Use dynamic voltage of 7 V
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    volts -> setControl(m_steerCharacterization.withVolts(volts)),
                    null,
                    this));

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle
     * HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on
     * importing the log to SysId.
     */
    @SuppressWarnings("unused")
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
            new SysIdRoutine.Config(
                    /* This is in radians per second², but SysId only supports "volts per second" */
                    Volts.of(Math.PI / 6).per(Second),
                    /* This is in radians per second, but SysId only supports "volts" */
                    Volts.of(Math.PI),
                    null, // Use default timeout (10 s)
                    // Log state with SignalLogger class
                    state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    output -> {
                        /* output is actually radians per second, but SysId only supports "volts" */
                        setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                        /* also log the requested output for SysId */
                        SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
                    },
                    null,
                    this));

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            PhotonCamera backRightCamera,
            PhotonCamera backLeftCamera,
            PhotonCamera frontCamera,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }

        this.backRightCamera = backRightCamera;
        this.backLeftCamera = backLeftCamera;
        this.frontCamera = frontCamera;

        backRightPoseEstimator = new PhotonPoseEstimator(VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                VisionConstants.BACK_RIGHT_CAMERA_POSE);
        frontPoseEstimator = new PhotonPoseEstimator(VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                VisionConstants.FRONT_CAMERA_POSE);
        backLeftPoseEstimator = new PhotonPoseEstimator(VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                VisionConstants.BACK_LEFT_CAMERA_POSE);

        pigeon2 = new Pigeon2(0, "Drivebase");
        headingController.enableContinuousInput(-Math.PI, Math.PI);

    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            PhotonCamera backRightCamera,
            PhotonCamera backLeftCamera,
            PhotonCamera frontCamera,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.frontCamera = frontCamera;
        this.backRightCamera = backRightCamera;
        this.backLeftCamera = backLeftCamera;

        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not
     * construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve
     *                                  drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz
     *                                  on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision
     *                                  calculation
     *                                  in the form [x, y, theta]ᵀ, with units in
     *                                  meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            PhotonCamera backRightCamera,
            PhotonCamera backLeftCamera,
            PhotonCamera frontCamera,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation,
                MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
        if (Utils.isSimulation()) {
            startSimThread();
        }
        this.backRightCamera = backRightCamera;
        this.backLeftCamera = backLeftCamera;
        this.frontCamera = frontCamera;
        headingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Returns a command that applies the specified control request to this swerve
     * drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    // Loops through all 4 modules and sets a state (an array of speed and position)
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, TunerConstants.kSpeedAt12Volts);
        ModuleRequest withState = new ModuleRequest();

        for (int i = 0; i < 4; i++) {
            getModule(i).apply(withState.withState(desiredStates[i]));
        }

    }

    // pulls the 2d state of the robot
    public Pose2d getPose() {
        return getStateCopy().Pose;
    }

    // factory for returning swerve auto commands
    public SwerveControllerCommand getAutoCommand(Trajectory trajectory) {

        // need to allow continues movement so the module actually works
        ProfiledPIDController thetaController = Constants.AutoConstants.thetaController;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SwerveControllerCommand(trajectory, this::getPose, Constants.Swerve.swerveKinematics,
                new HolonomicDriveController(
                        new PIDController(Constants.AutoConstants.kPXController, 0,
                                Constants.AutoConstants.kPXController / 2),
                        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                        thetaController),
                this::setModuleStates,
                this);

    }

    public void followTrajectory(SwerveSample sample) {
        // Get the current pose of the robot
        Pose2d pose = getPose();

        // Generate the next speeds for the robot
        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + xController.calculate(pose.getX(), sample.x),
                sample.vy + yController.calculate(pose.getY(), sample.y),
                sample.omega + headingController.calculate(pose.getRotation().getRadians(), sample.heading));


        // Apply the generated speeds
        this.setControl(driveFieldRelative(speeds));
    }

    public ChassisSpeeds getFieldRelativeChassisSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotRelativeChassisSpeeds(), getRotation());
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public SwerveRequest.RobotCentric getRobotCentricRequest() {
        return this.robotCentricRequest;
    }

    public SwerveRequest.FieldCentric getFieldCentricRequest() {
        return this.robotFieldCentric;
    }

    @Override
    public void periodic() {

        //Logger.recordOutput("SOTM/Measured Chassis Speeds", findChassisSpeeds());
        //Logger.recordOutput("SOTM/Chassis Speeds", getFieldRelativeChassisSpeeds());


        backRightResults = backRightCamera.getAllUnreadResults();
        backLeftResults = backLeftCamera.getAllUnreadResults();
        // frontResults = frontCamera.getAllUnreadResults();

        if (getCurrentCommand() != null) {
            Logger.recordOutput("Commands/RobotPose", getPose());
            Logger.recordOutput("Commands/Drivebase Command", getCurrentCommand().getName());
        }
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled.
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                        allianceColor == Alliance.Red
                                ? kRedAlliancePerspectiveRotation
                                : kBlueAlliancePerspectiveRotation);
                m_hasAppliedOperatorPerspective = true;
            });
        }

        // getEstimatedGlobalPose(frontPoseEstimator, frontCamera, frontResults);
        getEstimatedGlobalPose(backRightPoseEstimator, backRightCamera, backRightResults);
        getEstimatedGlobalPose(backLeftPoseEstimator, backLeftCamera, backLeftResults);

    }

    public ChassisSpeeds findChassisSpeeds() {
        Pose2d pose = getPose();
        Long timeDifference = secondSampleTime- firstSampleTime;
        double timeDifferenceSeconds = timeDifference / 1000000000.0;
        double filteredTimeDifference = 0.02; //chasisFilter.calculate(timeDifferenceSeconds);

        Logger.recordOutput("SOTM/secondSampleTime", secondSampleTime/1000000000.0);
        Logger.recordOutput("SOTM/firstSampleTime", firstSampleTime/1000000000.0);
        Logger.recordOutput("SOTM/timeDifference", filteredTimeDifference);

        return new ChassisSpeeds((pose.getX() - lastPose.getX()) / filteredTimeDifference,
                (pose.getY() - lastPose.getY()) / filteredTimeDifference,
                (pose.getRotation().minus(lastPose.getRotation())).getRadians() / filteredTimeDifference);
    }

    private MapleSimSwerveDrivetrain mapleSimSwerveDrivetrain = null;
    private void startSimThread() {
        mapleSimSwerveDrivetrain = new MapleSimSwerveDrivetrain(
            Seconds.of(kSimLoopPeriod),
            Pounds.of(135), // robot weight
            Inches.of(31.5), // bumper length
            Inches.of(31.25), // bumper width
            DCMotor.getKrakenX60(1), // drive motor type
            DCMotor.getKrakenX60(1), // steer motor type
            COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof, // wheel COF
            getModuleLocations(),
            getPigeon2(),
            getModules(),
            TunerConstants.FrontLeft,
            TunerConstants.FrontRight,
            TunerConstants.BackLeft,
            TunerConstants.BackRight);
    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(mapleSimSwerveDrivetrain::update);
    m_simNotifier.startPeriodic(kSimLoopPeriod);
}

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement
     *                                 in the form [x, y, theta]ᵀ, with units in
     *                                 meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds),
                visionMeasurementStdDevs);
    }

    public SwerveRequest driveFieldRelative(ChassisSpeeds speeds) {
        return m_applyFieldSpeeds.withSpeeds(speeds);
    }

    public double getPitch() {
        return pigeon2.getPitch().getValueAsDouble();
    }

    public double getYaw() {
        return pigeon2.getRotation2d().getDegrees();
    }

    public void getEstimatedGlobalPose(PhotonPoseEstimator poseEstimator, PhotonCamera camera,
            List<PhotonPipelineResult> results) {
        camera.setPipelineIndex(0);

        if (!results.isEmpty()) {
            List<PhotonTrackedTarget> targets = results.get(0).targets;
            PhotonPipelineResult result = results.get(0);
            //Logger.recordOutput("Vison/"+camera.getName() +  "targets", targets.toString());
            if (targets.size() == 1) {
                if (targets.get(0).poseAmbiguity < .1) {
                    addVisionMeasurement(
                            poseEstimator.estimateAverageBestTargetsPose(results.get(0)).get().estimatedPose.toPose2d(),
                            poseEstimator.estimateAverageBestTargetsPose(results.get(0)).get().timestampSeconds
                            );
                }
            } else if (targets.size() > 1 && poseEstimator.estimateCoprocMultiTagPose(results.get(0)).isPresent()) {
                addVisionMeasurement(
                        poseEstimator.estimateCoprocMultiTagPose(result).get().estimatedPose.toPose2d(),
                        poseEstimator.estimateCoprocMultiTagPose(result).get().timestampSeconds
                        );
            }
        }
    }

    private Matrix<N3, N1>  updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets, PhotonPoseEstimator estimator) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            return VisionConstants.SINGLE_TAG_STDDEV;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs =  VisionConstants.SINGLE_TAG_STDDEV;
            int numTags = 0;
            double avgDist = 0;


            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                return VisionConstants.SINGLE_TAG_STDDEV;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.MULTI_TAG_STDDEV;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                return estStdDevs;
            }
        }
    }

    @Override
    public void resetPose(Pose2d pose) {
        if (this.mapleSimSwerveDrivetrain != null)
            mapleSimSwerveDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
        Timer.delay(0.05); // Wait for simulation to update
        super.resetPose(pose);
    }

}
