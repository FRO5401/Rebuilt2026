// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import choreo.auto.AutoChooser;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
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
import frc.robot.Utils.HubTracker;
import frc.robot.Utils.MathHelp;
import frc.robot.Utils.RobotMode;
import frc.robot.Utils.TunableNumber;
import frc.robot.Utils.RobotMode.Mode;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.Swerve;
import frc.robot.Constants.Swerve.DriveType;

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

  //Leave these here god forbid we have to retune
  @SuppressWarnings("unused")
  private TunableNumber ShooterRPM = new TunableNumber("Shooter/RPM", 0,true);
  @SuppressWarnings("unused")
  private TunableNumber spindexerSpeed = new TunableNumber("Indexer/Spindexer Percent", 0, true);

  private PIDController thetaController = new PIDController(3, 0, 0);

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

  private Trigger gameShift;

  // Command Declaration
  private static Autos autos;
  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.Swerve.MaxSpeed * 0.01)
      .withRotationalDeadband(Constants.Swerve.MaxAngularRate * 0.01) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public static final RobotCentric robotCentricDrive = new RobotCentric()
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
        turret = new Turret(new TurretIOTalonFX(), drivetrain::getPose, drivetrain::findChassisSpeeds,
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

    gameShift = new Trigger(()-> HubTracker.getInstance().getShiftTimeCountdown() <= 5);


    configureAutoChooser();


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

    // driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    // driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    // driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    // driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    driver.rightBumper().whileTrue(Commands.runOnce(() -> shootingSpeed = 0.25));
    driver.rightBumper().whileFalse(Commands.runOnce(()-> shootingSpeed = 1));
 
    // // This is for the real robot
    turret.setDefaultCommand(turret.setSmartTarget());

    shooter.setDefaultCommand(shooter.setVelocity(()->RotationsPerSecond.of(0.0), intake::getDesiredAngle));

    // //this is for tuning
    // turret.setDefaultCommand(turret.runOnce(() -> turret.setTarget(FieldConstants.BLUE_HUB_TARGET)));

    // // this is for sim
    // turret.setDefaultCommand(turret.setSmartTarget()
    // .andThen(
    // Commands.runOnce(() ->
    // turret.updateFuel(MathHelp.findFlyWheelVelocity(turret.getPoseDifference())))));

    drivetrain.setDefaultCommand(drivetrain.applyRequest(()->getDriveRequest(DriveType.FIELD_CENTRIC)));

    driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    driver.x().whileTrue(drivetrain.applyRequest(()->getDriveRequest(DriveType.TRENCH)));
    driver.a().whileTrue(drivetrain.applyRequest(()->getDriveRequest(DriveType.BUMP)));


    // // //this is for tuning
    // operator.rightTrigger().whileTrue(new ParallelCommandGroup(Commands.repeatingSequence(shooter.setVelocity(
    //     () -> RotationsPerSecond
    //         .of(ShooterRPM.get()),
    //     intake::getDesiredAngle)),
    //     new SequentialCommandGroup(Commands.waitSeconds(.2),indexer.setIndexerCommand(() -> .8, () -> 11.0))));
 

    operator.y().onTrue(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE));
    operator.x().onTrue(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE * .75));
    operator.a().onTrue(intake.setPivotPositionCommand(0).andThen(intake.setInfeedVelocityCommand(0)));

    operator.leftTrigger().onTrue(intake.setInfeedVelocityCommand(IntakeConstants.INTAKE_SPEED));
    operator.leftTrigger().onFalse(intake.setInfeedVelocityCommand(0));

    operator.rightTrigger().whileTrue(new ParallelCommandGroup(Commands.repeatingSequence(shooter.setVelocity(
        () -> RotationsPerSecond
            .of(ShooterConstants.TREE_MAP.get(MathHelp.findDistance(turret.getPoseDifference()).baseUnitMagnitude())),
        intake::getDesiredAngle)),
        new SequentialCommandGroup(Commands.waitSeconds(.2),indexer.setIndexerCommand(() -> .8, () -> 11.0))));

    operator.rightTrigger().onFalse(indexer.setIndexerCommand(() -> 0.0, () -> 0.0));
    operator.rightBumper().onTrue(getAutonomousCommand()).onTrue(indexer.setIndexerCommand(()->-.5, ()->-4.0));
    operator.rightBumper().onFalse(indexer.setIndexerCommand(() -> 0.0, () -> 0.0));
    operator.start().whileTrue(shooter.setVelocity(
        () -> RotationsPerSecond
            .of(ShooterConstants.TREE_MAP.get(MathHelp.findDistance(turret.getPoseDifference()).baseUnitMagnitude())),
        intake::getDesiredAngle));

    gameShift.whileTrue((Commands.run(()-> operator.setRumble(RumbleType.kBothRumble, .5))));
    gameShift.whileFalse((Commands.run(()-> operator.setRumble(RumbleType.kBothRumble, 0))));


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

  public void configureAutoChooser(){
    autos = new Autos(drivetrain, turret, intake, shooter, indexer);
    autoChooser.addRoutine("DepotDoubleTrench", autos::leftDoubleTrenchAuto);
    autoChooser.addRoutine("DepotBumpSweep", autos::leftBumpAuto);
    autoChooser.addRoutine("DepotBumpNoSweep", autos::DepotNoSwipe);
    autoChooser.addRoutine("DepotSingleTrench", autos::leftSingleTrenchAuto);
    

    SmartDashboard.putData("Chooser", autoChooser);
  }

  /* Team 5000 Fuel Sim Set up */
  private void configureFuelSim() {
    if(RobotMode.currentMode != Mode.SIM) return;
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
    if(RobotMode.currentMode != Mode.SIM) return;
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

  private double getclosest90(Pose2d pose){
    return MathHelp.nearest90(pose.getRotation().getDegrees());
  }
  private double getclosest45(Pose2d pose){
    return MathHelp.nearest45(pose.getRotation().getDegrees());
  }

  private SwerveRequest getDriveRequest(DriveType driveType){
    switch(driveType){
      case BUMP -> {
        drive.withRotationalRate(
          Radians.convertFrom(
            thetaController.calculate(
              drivetrain.getPose().getRotation().getDegrees(), 
              getclosest90(drivetrain.getPose())), 
            Degrees)
        );
      }
      case TRENCH -> {
        drive.withRotationalRate(
          Radians.convertFrom(
            thetaController.calculate(
              drivetrain.getPose().getRotation().getDegrees(), 
              getclosest45(drivetrain.getPose())), 
            Degrees)
        );
      }
      case ROBOT_CENTRIC ->{
        return robotCentricDrive      
          .withVelocityX(shootingSpeed*-driver.getLeftY() * Constants.Swerve.MaxSpeed)
          .withVelocityY(shootingSpeed*-driver.getLeftX() * Constants.Swerve.MaxSpeed)
          .withRotationalRate(-driver.getRightX() * Constants.Swerve.MaxAngularRate)
          .withDesaturateWheelSpeeds(true);
      }
      default ->{
        drive.withRotationalRate(-driver.getRightX() * Constants.Swerve.MaxAngularRate);
      }
    }
    drive
      .withVelocityX(shootingSpeed*-driver.getLeftY() * Constants.Swerve.MaxSpeed)
      .withVelocityY(shootingSpeed*-driver.getLeftX() * Constants.Swerve.MaxSpeed)
      .withDesaturateWheelSpeeds(true);
    return drive;

  }
  
}
