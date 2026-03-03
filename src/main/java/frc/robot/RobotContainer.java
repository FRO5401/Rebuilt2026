// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import choreo.auto.AutoChooser;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.RobotDimensionConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Visulization;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Indexer.IndexerIOTalon;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Shooter.ShooterIOTalon;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.subsystems.Turret.TurretIOTalonFX;
import frc.robot.Utils.FuelSim;
import frc.robot.Utils.MathHelp;
import frc.robot.Utils.RobotMode;
import frc.robot.Utils.TunableNumber;
import frc.robot.Constants.ShooterConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private TunableNumber ShooterRPM = new TunableNumber("Shooter/RPM", 0);

  public static PhotonCamera backRightCamera = new PhotonCamera("backRightCamera");
  public static PhotonCamera backLeftCamera = new PhotonCamera("backLeftCamera");
  public static PhotonCamera frontCamera = new PhotonCamera("frontCamera");

  AutoChooser autoChooser = new AutoChooser("DoNothing");

  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(backRightCamera,
      backLeftCamera, frontCamera);
  // TurretIO turretIO = RobotBase.isReal() ? null : new TurretIOSim();
  // Turret turret = new Turret(turretIO, drivetrain::getPose,
  // drivetrain::getFieldRelativeChassisSpeeds);

  // ShooterIO shooterIO = RobotBase.isReal() ? null : new ShooterIOSim();
  // Shooter shooter = new Shooter(shooterIO);

  // IntakeIO intakeIO = RobotBase.isReal() ? null : new IntakeIOSim();
  // Intake intake = new Intake(new IntakeIOSim());

  // Subsystem Declaration
  private static double shootingSpeed = 1;
  private static Turret turret;
  private static Shooter shooter;
  private static Intake intake;
  private static Indexer indexer;
  private static Visulization visulization = null;

  // Command Declaration
  private static Autos autos;
  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.MaxSpeed * 0.01)
      .withRotationalDeadband(Constants.Swerve.MaxAngularRate * 0.01) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  @SuppressWarnings("unused")
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private CommandXboxController driver = new CommandXboxController(0);
  private CommandXboxController operator = new CommandXboxController(1);

  // Simulation Visulization
  FuelSim fuelSim;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Instantiation
    switch (RobotMode.currentMode) {
      case REAL:
        intake = new Intake(new IntakeIOTalonFX());
        shooter = new Shooter(new ShooterIOTalon());
        turret = new Turret(new TurretIOTalonFX(), drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds,
            intake::isNotStartingPose);
        // drivetrain::getFieldRelativeChassisSpeeds);
        indexer = new Indexer(new IndexerIOTalon());
        ShooterConstants.initializeTreeMap();
        break;

      case SIM:
        configureFuelSim();
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        turret = new Turret(new TurretIOSim(), drivetrain::getPose,
            drivetrain::getFieldRelativeChassisSpeeds, intake::isNotStartingPose);
        indexer = new Indexer(new IndexerIOTalon());

        visulization = new Visulization(fuelSim, drivetrain::getPose, turret, shooter, intake);
        configureFuelSimRobot(visulization::canIntake, visulization::intakeFuel);
        ShooterConstants.initializeTreeMap();
        break;

      default:
        intake = new Intake(null);
        shooter = new Shooter(null);
        turret = new Turret(null, drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds,
            intake::isNotStartingPose);
        indexer = new Indexer(null);
        break;
    }

    autos = new Autos(drivetrain, turret, intake, shooter, indexer);
    autoChooser.addRoutine("DepotDoubleTrench", autos::leftDoubleTrenchAuto);
    autoChooser.addRoutine("DepotBumpSweep", autos::leftBumpAuto);
    autoChooser.addRoutine("DepotSingleTrench", autos::leftSingleTrenchAuto);

    SmartDashboard.putData("Chooser", autoChooser);
    


    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    driver.rightBumper().whileTrue(Commands.runOnce(() -> shootingSpeed = 0.15));
    driver.rightBumper().whileFalse(Commands.runOnce(()-> shootingSpeed = 1));

    // // This is for the real robot
    // turret.setDefaultCommand(turret.setSmartTarget());

    // this is for tuning
    // turret.setDefaultCommand(turret.runOnce(() -> turret.setTarget(FieldConstants.BLUE_HUB_TARGET)));

    // this is for sim
    turret.setDefaultCommand(turret.setSmartTarget()
    .andThen(
    Commands.runOnce(() ->
    turret.updateFuel(MathHelp.findFlyWheelVelocity(turret.getPoseDifference())))));

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(shootingSpeed*-driver.getLeftY() * Constants.Swerve.MaxSpeed)
            .withVelocityY(shootingSpeed*-driver.getLeftX() * Constants.Swerve.MaxSpeed)
            .withRotationalRate(
                -driver.getRightX() * Constants.Swerve.MaxAngularRate)
            .withDesaturateWheelSpeeds(true)));

    driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    //This is for real
    shooter.setDefaultCommand(shooter.setVelocity(
        () -> RotationsPerSecond
            .of(ShooterConstants.TREE_MAP.get(MathHelp.findDistance(turret.getPoseDifference()).baseUnitMagnitude())),
        intake::getDesiredAngle));

    //this is for sim
      // shooter.setDefaultCommand(shooter.setVelocity(
      //   () -> RotationsPerSecond
      //       .of(ShooterRPM.get()),
      //   intake::getDesiredAngle));
 

    operator.y().onTrue(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE));
    operator.x().onTrue(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE / 2.0));
    operator.a().onTrue(intake.setPivotPositionCommand(0).andThen(intake.setInfeedVelocityCommand(0)));

    operator.leftTrigger().onTrue(intake.setInfeedVelocityCommand(IntakeConstants.INTAKE_SPEED));
    operator.leftBumper().onTrue(intake.setInfeedVelocityCommand(0));

    operator.rightTrigger().whileTrue(indexer.setIndexerCommand(() -> 4.0, () -> 11.0));
    operator.rightTrigger().onFalse(indexer.setIndexerCommand(() -> 0.0, () -> 0.0));
    operator.rightBumper().onTrue(getAutonomousCommand()).onTrue(indexer.setIndexerCommand(()->-4.0, ()->-4.0));
    operator.rightBumper().onFalse(indexer.setIndexerCommand(() -> 0.0, () -> 0.0));




  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return Commands.none();
    return autoChooser.selectedCommand();
  }

  /* Team 5000 Fuel Sim Set up */
  private void configureFuelSim() {
    fuelSim = new FuelSim("Fuel-Pose");
    fuelSim.spawnStartingFuel();
    fuelSim.enableAirResistance();

    fuelSim.start();
    SmartDashboard.putData(Commands.runOnce(() -> {
      fuelSim.clearFuel();
      fuelSim.spawnStartingFuel();
    })
        .withName("Reset Fuel")
        .ignoringDisable(true));
  }

  private void configureFuelSimRobot(BooleanSupplier ableToIntake, Runnable intakeCallback) {
    fuelSim.registerRobot(
        RobotDimensionConstants.WIDTH_WBUMPERS,
        RobotDimensionConstants.LENGTH_WBUMPERS,
        RobotDimensionConstants.HEIGHT_OF_BUMPERS,
        drivetrain::getPose,
        drivetrain::getFieldRelativeChassisSpeeds);
    fuelSim.registerIntake(
        RobotDimensionConstants.INTAKE_XMIN,
        RobotDimensionConstants.INTAKE_XMAX,
        RobotDimensionConstants.INTAKE_YMIN,
        RobotDimensionConstants.INTAKE_YMAX,
        () -> intake.isIntakeDeployed() && ableToIntake.getAsBoolean(),
        intakeCallback);
  }
}
