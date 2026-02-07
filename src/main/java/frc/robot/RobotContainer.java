// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretIO;
import frc.robot.subsystems.Turret.TurretIOSim;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  CommandXboxController controller = new CommandXboxController(0);

  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  TurretIO turretIO = RobotBase.isReal() ? null : new TurretIOSim();

  Turret turret = new Turret(turretIO, drivetrain::getPose, drivetrain::getFieldRelativeChassisSpeeds);

      /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.Swerve.MaxSpeed * 0.01)
            .withRotationalDeadband(Constants.Swerve.MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  Autos autos = new Autos(drivetrain, turret);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    controller.leftBumper().onTrue(Commands.runOnce(()->turret.setTarget(new Pose2d(4.5, 4, new Rotation2d()))));
    controller.rightBumper().onTrue(Commands.runOnce(()->turret.setTarget(new Pose2d(11.9, 4, new Rotation2d()))));
    controller.x().onTrue(Commands.runOnce(()->turret.setTarget(new Pose2d(2, 2, new Rotation2d()))));



    drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-controller.getLeftY() * Constants.Swerve.MaxSpeed) 
                        .withVelocityY(-controller.getLeftX() * Constants.Swerve.MaxSpeed)                                                                          
                        .withRotationalRate(
                                -controller.getRightX() * Constants.Swerve.MaxAngularRate)
                                //SwerveUtils.rotationPoint(new Rotation2d(-driver.getRightY(), -driver.getRightX()).getDegrees(), drivetrain.getYaw()) * Constants.Swerve.MaxAngularRate * elevator.getSpeedModifier()) 
                .withDesaturateWheelSpeeds(true)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return autos.testAuto().cmd();
  }
}
