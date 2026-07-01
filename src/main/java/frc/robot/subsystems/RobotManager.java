// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldZones;
import frc.robot.Utils.HubTracker;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Turret.Turret;

public class RobotManager extends SubsystemBase {
  private final CommandSwerveDrivetrain drivetrain;
  private final Turret turret;
  private final Hood hood;
  private final Intake intake;
  private final Indexer indexer;
  private final CANdleSystem candle;

  private final HubTracker hubTracker = HubTracker.getInstance();

  private final Trigger trenchZone;

  // TODO: Find Time for the hood to lower from max to start
  private final Time hoodDropTime = Seconds.of(0.5);

  /** Creates a new RobotManager. */
  public RobotManager(
      CommandSwerveDrivetrain drivetrain,
      Turret turret,
      Hood hood,
      Intake intake,
      Indexer indexer,
      CANdleSystem candle) {
    this.drivetrain = drivetrain;
    this.turret = turret;
    this.hood = hood;
    this.intake = intake;
    this.indexer = indexer;
    this.candle = candle;

    trenchZone =
        FieldZones.PREDICTIVE_TRENCH_GROUP.willContain(
            drivetrain::getPose, drivetrain.getFieldRelativeChassisSpeeds(), hoodDropTime);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
