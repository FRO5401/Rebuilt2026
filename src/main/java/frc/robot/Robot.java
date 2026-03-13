// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldZones;
import frc.robot.Utils.HubTracker;
import frc.robot.Utils.RobotMode;

/**
 * The methods in this class are called automatically corresponding to each
 * mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the
 * package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    Logger.recordMetadata("Rebuilt2026","TurretTest"); // Set a metadata value

    switch (RobotMode.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();



  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // TODO: Debugging purposes Remove at some point before comp

    Logger.recordOutput("Current Shift", HubTracker.getInstance().getCurrentShift());
    Logger.recordOutput("Is hub Active", HubTracker.getInstance().isHubActive());
    Logger.recordOutput("Match Time", HubTracker.getInstance().getMatchTime());

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    HubTracker.getInstance().stopMatchTimer();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    HubTracker.getInstance().initalizeMatchTimer();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    HubTracker.getInstance().initalizeMatchTimer();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    //HubTracker.getInstance().initalizeMatchTimer();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    m_robotContainer.fuelSim.updateSim();
    Logger.recordOutput("Zones/Field", FieldZones.FIELD_ZONE.getCorners());
    Logger.recordOutput("Zones/Blue", FieldZones.BLUE_ZONE.getCorners());
    Logger.recordOutput("Zones/Red", FieldZones.RED_ZONE.getCorners());
    Logger.recordOutput("Zones/Nuetral", new Translation2d[][]{FieldZones.NUETRAL_BLUE_DEPO.getCorners(), FieldZones.NUETRAL_BLUE_OUTPOST.getCorners()});
    Logger.recordOutput("Zones/Trenches", new Translation2d[][]{FieldZones.BLUE_TRENCH_DEPO_ZONE.getCorners(), FieldZones.BLUE_TRENCH_OUTPOST_ZONE.getCorners(),FieldZones.RED_TRENCH_DEPO_ZONE.getCorners(), FieldZones.RED_TRENCH_OUTPOST_ZONE.getCorners()});
    Logger.recordOutput("Zones/Bumps", new Translation2d[][]{FieldZones.BLUE_BUMP_DEPO_ZONE.getCorners(), FieldZones.BLUE_BUMP_OUTPOST_ZONE.getCorners(),FieldZones.RED_BUMP_DEPO_ZONE.getCorners(), FieldZones.RED_BUMP_OUTPOST_ZONE.getCorners()});
    Logger.recordOutput("Zones/Turtle", new Translation2d[][]{FieldZones.BLUE_TURTLE_ZONE.getCorners(), FieldZones.RED_TURTLE_ZONE.getCorners()});
    
  }

}