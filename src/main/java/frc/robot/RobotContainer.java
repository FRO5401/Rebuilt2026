// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.Constants.MathConstants;
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
import frc.robot.Constants.ShooterConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static PhotonCamera backRightCamera = new PhotonCamera("backRightCamera");
  public static PhotonCamera backLeftCamera = new PhotonCamera("backLeftCamera");
  public static PhotonCamera frontCamera = new PhotonCamera("frontCamera");

  

<<<<<<< HEAD
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(backRightCamera, backLeftCamera, frontCamera);
  //TurretIO turretIO = RobotBase.isReal() ? null : new TurretIOSim();
  //Turret turret = new Turret(turretIO, drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds);

  //ShooterIO shooterIO = RobotBase.isReal() ? null : new ShooterIOSim();
  //Shooter shooter = new Shooter(shooterIO);

  //IntakeIO intakeIO = RobotBase.isReal() ? null : new IntakeIOSim();
  //Intake intake = new Intake(new IntakeIOSim());
=======
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(frontRightCamera, frontLeftCamera, backCamera);
>>>>>>> d31c7bb7d4e593f96c6d4245b152969267ae5884

  //TurretIO turretIO = RobotBase.isReal() ? null : new TurretIOSim();
  //Turret turret = new Turret(turretIO, drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds);

  //ShooterIO shooterIO = RobotBase.isReal() ? null : new ShooterIOSim();
  //Shooter shooter = new Shooter(shooterIO);

  //IntakeIO intakeIO = RobotBase.isReal() ? null : new IntakeIOSim();
  //Intake intake = new Intake(new IntakeIOSim());

  //  Subsystem Declaration
  private static Turret turret;
  private static Shooter shooter;
  private static Intake intake;
  private static Indexer indexer;
  private static Visulization visulization = null;

  //  Command Declaration
  private static Autos autos;

      /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MaxSpeed * 0.01)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    
    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private CommandXboxController controller = new CommandXboxController(0);

    // Simulation Visulization 
    FuelSim fuelSim;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {
    // Instantiation 
    switch(RobotMode.currentMode){
      case REAL: 
        intake = new Intake(new IntakeIOTalonFX());
        shooter = new Shooter(new ShooterIOTalon());
        turret = new Turret(new TurretIOTalonFX(), drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds);
        indexer = new Indexer(new IndexerIOTalon());
        ShooterConstants.initializeTreeMap();
        break;

      case SIM:
        configureFuelSim();
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        turret = new Turret(new TurretIOSim(), drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds);
        indexer = new Indexer(new IndexerIOTalon());
        visulization = new Visulization(fuelSim, drivetrain::getPose, turret, shooter, intake);
        configureFuelSimRobot(visulization::canIntake, visulization::intakeFuel);
        ShooterConstants.initializeTreeMap();
        break;

      default:
        intake = new Intake(null);
        shooter = new Shooter(null);
        turret = new Turret(null, drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds);
        indexer = new Indexer(null);
        break;
    }

    autos = new Autos(drivetrain, turret, intake, shooter);

    // Configure the trigger bindings
    configureBindings();
  
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    turret.setDefaultCommand(turret.setSmartTarget().andThen(Commands.runOnce(() -> turret.updateFuel(MetersPerSecond.of(shooter.getVelocity().in(RotationsPerSecond) * (Math.PI*MathConstants.FLY_WHEEL_DIAMETER.in(Meters)))))));

    shooter.setDefaultCommand(
      shooter.setVelocity(() -> MathHelp.findFlyWheelRPM(MathHelp.findFlyWheelVelocity(turret.getPoseDifference())))
    );

    drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-controller.getLeftY() * Constants.Swerve.MaxSpeed) 
                        .withVelocityY(-controller.getLeftX() * Constants.Swerve.MaxSpeed)                                                                          
                        .withRotationalRate(
                                -controller.getRightX() * Constants.Swerve.MaxAngularRate)
                .withDesaturateWheelSpeeds(true)));

    controller.y().onTrue(intake.setPivotPositionCommand(90));
    controller.x().onTrue(intake.setPivotPositionCommand(45));
    controller.a().onTrue(intake.setPivotPositionCommand(0).andThen(intake.setInfeedVelocityCommand(0)));

    intake.setDefaultCommand(Commands.run(()->intake.setInfeedVelocity(controller.getRightTriggerAxis()), intake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autos.testAuto().cmd().withName("Auto");
  }

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
            Inches.of(34.56),
            Inches.of(34.560082),
            Inches.of(5.858080),
            drivetrain::getPose,
            drivetrain::getFieldRelativeChassisSpeeds
    );
    fuelSim.registerIntake(
            Inches.of(34.560082).div(2).in(Meters),
            Inches.of(34.560082).div(2).plus(Inches.of(8.345)).in(Meters),
            -Inches.of(34.56).div(2).in(Meters),
            Inches.of(34.56).div(2).in(Meters),
            ()-> intake.isIntakeDeployed() && ableToIntake.getAsBoolean(),
            intakeCallback
    );
  }
}
