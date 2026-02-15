// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeIOSim;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterIO;
import frc.robot.subsystems.Shooter.ShooterIOSim;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOSim;
import frc.robot.Utils.MathHelp;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  TurretIO turretIO = RobotBase.isReal() ? null : new TurretIOSim();
  Turret turret = new Turret(turretIO, drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds);

  ShooterIO shooterIO = RobotBase.isReal() ? null : new ShooterIOSim();
  Shooter shooter = new Shooter(shooterIO);


      /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MaxSpeed * 0.01)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    @SuppressWarnings("unused")
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  Autos autos = new Autos(drivetrain, turret);

  Intake intake = new Intake(new IntakeIOSim());
  CommandXboxController controller = new CommandXboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    intake.setDefaultCommand(Commands.run(()->intake.setInfeedVelocity(controller.getRightTriggerAxis()), intake));
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
    controller.a().onTrue(Commands.runOnce(()->intake.setPivotPosition(90), intake));
   controller.b().onTrue(Commands.runOnce(()->intake.setPivotPosition(45), intake));
   controller.x().onTrue(Commands.runOnce(()-> intake.setIntake(0, 0), intake));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autos.testAuto().cmd().withName("Auto");
  }
}
