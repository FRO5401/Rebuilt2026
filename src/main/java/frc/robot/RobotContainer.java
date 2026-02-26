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

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.subsystems.Intake.IntakeIOTalonFX;
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
import frc.robot.Utils.RobotMode.Mode;
import frc.robot.Constants.ShooterConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static PhotonCamera frontRightCamera = new PhotonCamera("frontRightCamera");
  public static PhotonCamera frontLeftCamera = new PhotonCamera("frontLeftCamera");
  public static PhotonCamera backCamera = new PhotonCamera("backCamera");

  

  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(frontRightCamera, frontLeftCamera, backCamera);

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
        resetSimulation();
        intake = new Intake(new IntakeIOSim());
        shooter = new Shooter(new ShooterIOSim());
        turret = new Turret(new TurretIOSim(), drivetrain::getSimPose, drivetrain::getFieldRelativeChassisSpeeds);
        indexer = new Indexer(new IndexerIOTalon());
        visulization = new Visulization(drivetrain::getSimPose, turret, shooter, intake);
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
    drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-controller.getLeftY() * Constants.Swerve.MaxSpeed) 
                        .withVelocityY(-controller.getLeftX() * Constants.Swerve.MaxSpeed)                                                                          
                        .withRotationalRate(
                                -controller.getRightX() * Constants.Swerve.MaxAngularRate)
                .withDesaturateWheelSpeeds(true)));

    turret.setDefaultCommand(turret.setSmartTarget().andThen(Commands.runOnce(() -> turret.updateFuel(MetersPerSecond.of(shooter.getVelocity().in(RotationsPerSecond) * (Math.PI*MathConstants.FLY_WHEEL_DIAMETER.in(Meters)))))));

    shooter.setDefaultCommand(
    shooter.setVelocity(() -> MathHelp.findFlyWheelRPM(MathHelp.findFlyWheelVelocity(turret.getPoseDifference())))
    );

    controller.y().onTrue(intake.setPivotPositionCommand(90));
    controller.x().onTrue(intake.setPivotPositionCommand(45));
    controller.a().onTrue(intake.setPivotPositionCommand(0).andThen(intake.setInfeedVelocityCommand(0)));

    controller.leftTrigger().onTrue(intake.setInfeedVelocityCommand(IntakeConstants.INTAKE_SPEED));
    controller.leftBumper().onTrue(intake.setInfeedVelocityCommand(0));


    /* THESE ARE ALL FOR PID TUNING AND SHOULD NOT BE USED ON THE ROBOT */
    shooter.setDefaultCommand(shooter.setVelocity(()->RotationsPerSecond.of(120*controller.getRightTriggerAxis())));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autos.testAuto().cmd().withName("Auto");
  }

  public void resetSimulation(){
    if(RobotMode.currentMode != Mode.SIM) return;
    drivetrain.resetPose(new Pose2d(2, 2, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation(){
    if(RobotMode.currentMode != Mode.SIM) return;
    SimulatedArena.getInstance().simulationPeriodic();
    if(drivetrain.mapleSimSwerveDrivetrain != null) {
      Logger.recordOutput("Visulization/Maple Sim Robot Pose", drivetrain.mapleSimSwerveDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
    }
    Logger.recordOutput("FieldSimulation/FuelPositions", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
  }
}
