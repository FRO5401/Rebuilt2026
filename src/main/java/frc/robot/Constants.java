// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Current;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class TurretConstants{

    
  public static final Current STATOR_LIMIT = Amps.of(120);
  public static final Current SUPPLY_LIMIT = Amps.of(20);
  public static final double GEAR_RATIO = 25; 

  public static final double KP = 0.5;
  public static final double KI = 0.0;
  public static final double KD = 0.0;
  
  public static final double KP_SIM = 1000;
  public static final double KI_SIM = 0.0;
  public static final double KD_SIM = 0.0;


  public static final MotorOutputConfigs OUTPUT_CONFIG = new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);

      public static final FeedbackConfigs FEEDBACK_CONFIG = new FeedbackConfigs()
      .withSensorToMechanismRatio(GEAR_RATIO);

      
    public static final CurrentLimitsConfigs CURRENT_LIMITS_CONFIG = new CurrentLimitsConfigs()
      .withStatorCurrentLimit(STATOR_LIMIT)
      .withSupplyCurrentLimit(SUPPLY_LIMIT);

      public static final Slot0Configs CLOSED_LOOP = new Slot0Configs()
      .withKP(0)
      .withKI(0)
      .withKD(0)
      .withKA(0)
      .withKV(0)
      .withKG(0);

      public static final TalonFXConfiguration CONFIG = new TalonFXConfiguration()
      .withSlot0(CLOSED_LOOP)
      .withCurrentLimits(CURRENT_LIMITS_CONFIG)
      .withMotorOutput(OUTPUT_CONFIG)
      .withFeedback(FEEDBACK_CONFIG);

      /** The transform of the center of the robot to the position of the turret
      * This is used to calculate the required velocity of the shot
      */
      Transform2d TURRET_TRANSFORM = new Transform2d(Inches.of(0), Inches.of(0), new Rotation2d(0));
  }
}
