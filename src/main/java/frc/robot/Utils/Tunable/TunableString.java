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
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import java.util.function.Supplier;

/**
 * Tunable wrapper for {@link String} stored in NetworkTables.
 *
 * <p>When tuning mode is enabled, the value is published to NetworkTables and can be modified live
 * through dashboards such as AdvantageScope, Shuffleboard, or Elastic. When tuning mode is
 * disabled, this class simply returns the provided default value with no NetworkTables overhead.
 *
 * <p>Implements {@link Supplier} so it can be used directly in control code.
 */
public class TunableString extends TunableBase<String> implements Supplier<String> {
  private static final NetworkTable DATA_TABLE = INSTANCE.getTable(DIRECTORY);

  private NetworkTable netTable;

  private StringSubscriber sub;
  private StringPublisher pub;

  /**
   * Creates a tunable String without a default value.
   *
   * <p>{@link #setDefaultValue(String)} must be called before the value can be used.
   *
   * @param m_key NetworkTables key for this value.
   */
  public TunableString(String m_key) {
    super(m_key);
  }

  /**
   * Creates a tunable String with the specified default value.
   *
   * @param m_key NetworkTables key.
   * @param m_defaultValue Default String.
   * @param m_disableTuning If true, this value will never be tunable even when robot tuning mode is
   *     enabled.
   */
  public TunableString(String m_key, String m_defaultValue) {
    super(m_key, m_defaultValue);
  }

  /**
   * Creates a tunable String with an option to disable tuning.
   *
   * @param m_key NetworkTables key.
   * @param m_defaultValue Default String.
   * @param m_disableTuning If true, this value will never be tunable even when robot tuning mode is
   *     enabled.
   */
  public TunableString(String m_key, String m_defaultValue, boolean m_disableTuning) {
    super(m_key, m_defaultValue, m_disableTuning);
  }

  /**
   * Initializes NetworkTables entries for each component of String.
   *
   * <p>Creates a subtable and binds each field to its own TunableString entry.
   */
  @Override
  protected void initTunable() {
    pub = DATA_TABLE.getStringTopic(key).publish();
    pub.set(defaultValue);

    sub = DATA_TABLE.getStringTopic(key).subscribe(defaultValue);
  }

  /**
   * Returns the current String value.
   *
   * <p>If tuning is enabled, values are read live from NetworkTables. Otherwise, the stored default
   * String is returned.
   *
   * @return Current String.
   */
  @Override
  public String getValue() {
    if (!hasDefault) {
      return "";
    } else {
      return isTuningEnabled() ? sub.get() : defaultValue;
    }
  }

  /**
   * Supplier implementation for functional usage.
   *
   * @return Current String value.
   */
  @Override
  public String get() {
    return getValue();
  }
}
