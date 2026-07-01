// Copyright (c) 2026 Bensalem High School Fightin' Robotic Owls
// https://github.com/FRO5401
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Contributors:
//  2026 Soccertoad Ben Reinert

package frc.robot.Utils.Tunable;

import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Utils.Gains;
import frc.robot.Utils.Gains.GravityType;
import java.util.function.Supplier;

public class TunableGains extends TunableBase<Gains> implements Supplier<Gains> {

  private NetworkTable netTable;

  private TunableNumber kP;
  private TunableNumber kI;
  private TunableNumber kD;
  private TunableNumber kS;
  private TunableNumber kV;
  private TunableNumber kA;
  private TunableNumber kG;

  private GravityType gravityType = GravityType.ELEVATOR_STATIC;

  public TunableGains(String m_key) {
    super(m_key);
  }

  public TunableGains(String m_key, Gains m_defaultValue) {
    super(m_key, m_defaultValue);
  }

  public TunableGains(String m_key, Gains m_defaultValue, GravityType gravityType) {
    super(m_key, m_defaultValue);
    this.gravityType = gravityType;
  }

  public TunableGains(String m_key, Gains m_defaultValue, boolean m_disableTuning) {
    super(m_key, m_defaultValue, m_disableTuning);
  }

  public TunableGains(
      String m_key, Gains m_defaultValue, boolean m_disableTuning, GravityType gravityType) {
    super(m_key, m_defaultValue, m_disableTuning);
    this.gravityType = gravityType;
  }

  @Override
  protected void initTunable() {
    netTable = INSTANCE.getTable(key);

    kP = new TunableNumber("kP", defaultValue.kP, netTable);
    kI = new TunableNumber("kI", defaultValue.kI, netTable);
    kD = new TunableNumber("kD", defaultValue.kD, netTable);
    kS = new TunableNumber("kS", defaultValue.kS, netTable);
    kV = new TunableNumber("kV", defaultValue.kV, netTable);
    kA = new TunableNumber("kA", defaultValue.kA, netTable);
    kG = new TunableNumber("kG", defaultValue.kG, netTable);
  }

  @Override
  public Gains getValue() {
    if (!hasDefault) {
      return Gains.ZEROED_GAINS;
    } else {
      return isTuningEnabled()
          ? new Gains(
              kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kA.get(), kG.get(), gravityType)
          : defaultValue;
    }
  }

  @Override
  public Gains get() {
    return getValue();
  }
}
