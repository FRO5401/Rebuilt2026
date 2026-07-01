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
import frc.robot.Utils.FireControl.ShotData;
import java.util.function.Supplier;

/**
 * Tunable wrapper for {@link ShotData} stored in NetworkTables.
 *
 * <p>Breaks a composite ShotData object into individually tunable fields: flywheel RPS, hood
 * rotation, and time of flight.
 *
 * <p>Implements {@link Supplier} so it can be used directly in control code.
 */
public class TunableShotData extends TunableBase<ShotData> implements Supplier<ShotData> {

  private TunableNumber flywheelRPS;
  private TunableNumber hoodRotations;
  private TunableNumber tof;

  private NetworkTable netTable;

  /**
   * Creates a tunable ShotData without a default value.
   *
   * <p>{@link #setDefaultValue(ShotData)} must be called before the value can be used.
   *
   * @param m_key NetworkTables key for this value.
   */
  public TunableShotData(String m_key) {
    super(m_key);
  }

  /**
   * Creates a tunable ShotData with the specified default value.
   *
   * @param m_key NetworkTables key.
   * @param m_defaultValue Default ShotData.
   * @param m_disableTuning If true, this value will never be tunable even when robot tuning mode is
   *     enabled.
   */
  public TunableShotData(String m_key, ShotData m_defaultValue) {
    super(m_key, m_defaultValue);
  }

  /**
   * Creates a tunable ShotData with an option to disable tuning.
   *
   * @param m_key NetworkTables key.
   * @param m_defaultValue Default ShotData.
   * @param m_disableTuning If true, this value will never be tunable even when robot tuning mode is
   *     enabled.
   */
  public TunableShotData(String m_key, ShotData m_defaultValue, boolean m_disableTuning) {
    super(m_key, m_defaultValue, m_disableTuning);
  }

  /**
   * Initializes NetworkTables entries for each component of ShotData.
   *
   * <p>Creates a subtable and binds each field to its own TunableNumber entry.
   */
  @Override
  protected void initTunable() {
    this.netTable = INSTANCE.getTable(key);
    this.flywheelRPS = new TunableNumber("RPS", defaultValue.rps(), netTable);
    this.hoodRotations = new TunableNumber("HoodRotations", defaultValue.hoodRotations(), netTable);
    this.tof = new TunableNumber("ToF", defaultValue.tof(), netTable);
  }

  /**
   * Returns the current ShotData value.
   *
   * <p>If tuning is enabled, values are read live from NetworkTables. Otherwise, the stored default
   * ShotData is returned.
   *
   * @return Current ShotData.
   */
  @Override
  public ShotData getValue() {
    if (!hasDefault) {
      return ShotData.ZEROED;
    } else {
      return isTuningEnabled()
          ? new ShotData(flywheelRPS.get(), hoodRotations.get(), tof.get())
          : defaultValue;
    }
  }

  /**
   * Supplier implementation for functional usage.
   *
   * @return Current ShotData value.
   */
  @Override
  public ShotData get() {
    return getValue();
  }
}
