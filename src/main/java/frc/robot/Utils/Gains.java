// Copyright (c) 2026 Bensalem High School Fightin' Robotic Owls
// https://github.com/FRO5401
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Contributors:
//  2026 Soccertoad Ben Reinert

package frc.robot.Utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;

public class Gains {
  public double kP = 0;
  public double kI = 0;
  public double kD = 0;
  public double kS = 0;
  public double kV = 0;
  public double kA = 0;
  public double kG = 0;
  public GravityType gravityType = GravityType.ELEVATOR_STATIC;

  public Gains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      GravityType gravityType) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
    this.gravityType = gravityType;
  }

  public Gains() {
    this(0, 0, 0, 0, 0, 0, 0, GravityType.ELEVATOR_STATIC);
  }

  public Gains withPID(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    return this;
  }

  public Gains withFeedforward(double kS, double kV, double kA, double kG) {
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
    this.kG = kG;
    return this;
  }

  public Gains withGravityType(GravityType gravityType) {
    this.gravityType = gravityType;
    return this;
  }

  public enum GravityType {
    ELEVATOR_STATIC,
    ARM_COSINE
  }

  public Slot0Configs toSlot0Configs() {
    var config =
        new Slot0Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withGravityType(GravityTypeValue.Elevator_Static);

    if (gravityType.equals(GravityType.ARM_COSINE))
      config.withGravityType(GravityTypeValue.Arm_Cosine);

    return config;
  }

  public Slot1Configs toSlot1Configs() {
    var config =
        new Slot1Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withGravityType(GravityTypeValue.Elevator_Static);

    if (gravityType.equals(GravityType.ARM_COSINE))
      config.withGravityType(GravityTypeValue.Arm_Cosine);

    return config;
  }

  public Slot2Configs toSlot2Configs() {
    var config =
        new Slot2Configs()
            .withKP(kP)
            .withKI(kI)
            .withKD(kD)
            .withKS(kS)
            .withKV(kV)
            .withKA(kA)
            .withGravityType(GravityTypeValue.Elevator_Static);

    if (gravityType.equals(GravityType.ARM_COSINE))
      config.withGravityType(GravityTypeValue.Arm_Cosine);

    return config;
  }

  public PIDController toPIDController() {
    return new PIDController(kP, kI, kD);
  }

  public ElevatorFeedforward toElevatorFeedforward() {
    return new ElevatorFeedforward(kS, kG, kV, kA);
  }

  public ArmFeedforward toArmFeedforward() {
    return new ArmFeedforward(kS, kG, kV, kA);
  }

  public static final Gains ZEROED_GAINS = new Gains();
}
