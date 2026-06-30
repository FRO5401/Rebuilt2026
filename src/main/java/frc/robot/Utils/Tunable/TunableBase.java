// Copyright (c) 2026 Bensalem High School Fightin' Robotic Owls
// https://github.com/FRO5401
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Contributors:
//  2026 Soccertoad Ben Reinert

package frc.robot.Utils.Tunable;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Utils.RobotMode;

// Java Docs as of 6/30/26 were written by AI
// All logic and code was created without AI.
// This takes inspiration from 2026 Littleton Robotics LoggedTunableNumber

/**
 * Base class for tunable values backed by NetworkTables.
 *
 * <p>
 * This class provides common functionality for creating tunable robot constants that can be
 * modified at runtime while the robot is in tuning mode. Subclasses are responsible for
 * implementing the NetworkTables publisher/subscriber logic for their specific value type.
 *
 * <p>
 * Each tunable stores a default value that is used whenever tuning is disabled. When tuning is
 * enabled, subclasses publish the default value and retrieve updates from NetworkTables.
 *
 * @param <V> The type of value stored by this tunable.
 */
public abstract class TunableBase<V> {
  protected static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();

  protected static final String DIRECTORY = "/Tunable";
  protected String key;

  protected V defaultValue;
  protected boolean hasDefault = false;

  protected Map<Integer, V> lastValueMap = new HashMap<>();

  protected boolean isTuningDisabled = false;
 
  /**
   * Returns whether tuning is currently active.
   *
   * <p>Tuning is active only when robot mode allows it and this tunable is not disabled.
   */
  protected boolean isTuningEnabled(){
    return RobotMode.isTuningMode ? !isTuningDisabled : false;
  }
  
  /**
   * Creates a tunable value with the given key.
   *
   * @param m_key Name of the tunable entry relative to the tunable directory.
   */
  protected TunableBase(String m_key) {
    this.key = formatKey(m_key);
  }

  /**
   * Creates a tunable value with the specified default value.
   *
   * @param m_key Name of the tunable entry.
   * @param m_defaultValue Default value when tuning is disabled.
   */
  protected TunableBase(String m_key, V m_defaultValue) {
    this(m_key);
    setDefaultValue(m_defaultValue);
  }

  /**
   * Creates a tunable value with an option to disable tuning.
   *
   * @param m_key Name of the tunable entry.
   * @param m_defaultValue Default value.
   * @param m_disableTuning If true, this value will never be tunable.
   */
  protected TunableBase(String m_key, V m_defaultValue, boolean m_disableTuning) {
    this(m_key);
    setDefaultValue(m_defaultValue);
    this.isTuningDisabled = m_disableTuning;
  }

  /**
   * Returns the current value of this tunable.
   *
   * <p>
   * If tuning is enabled, subclasses should return the current value from NetworkTables. Otherwise,
   * they should return the stored default value.
   *
   * @return The current value.
   */
  public abstract V getValue();

  /**
   * Initializes the NetworkTables publisher and subscriber for this tunable.
   *
   * <p>
   * This method is called automatically the first time a default value is assigned while tuning is
   * enabled.
   */
  protected abstract void initTunable();

  /**
   * Sets the default value of this tunable.
   *
   * <p>
   * The default value may only be assigned once. Subsequent calls have no effect.
   *
   * @param m_defaultValue Default value to store.
   */
  public void setDefaultValue(V m_defaultValue) {
    if (!hasDefault) {
      this.hasDefault = true;
      this.defaultValue = m_defaultValue;

      if (isTuningEnabled()) initTunable();
    }

  }

  /**
   * Determines whether the value has changed since the previous check for the given caller.
   *
   * <p>
   * The supplied identifier allows multiple independent callers to track changes to the same
   * tunable without interfering with one another.
   *
   * @param id Unique identifier for the caller. "hashCode()"
   * @return {@code true} if the value has changed since the previous call for this identifier.
   */
  public boolean hasChanged(int id) {
    if (!isTuningEnabled()) return false;

    V currentValue = getValue();
    V lastValue = lastValueMap.get(id);
    if (lastValue == null || !lastValue.equals(currentValue)) {
      lastValueMap.put(id, currentValue);
      return true;
    }

    return false;
  }

  /**
   * Determines whether any of the supplied tunables have changed for the given caller.
   *
   * @param id Unique identifier for the caller. "hashCode()"
   * @param tunables Tunables to check.
   * @return {@code true} if any supplied tunable has changed.
   */
  public static boolean hasChanged(int id, TunableBase<?>... tunables) {
    if (Arrays.stream(tunables).anyMatch(tunable -> tunable.hasChanged(id))) {
      return true;
    }
    return false;
  }

  private static String formatKey(String m_key){
    if (m_key.startsWith("/")) {
      return DIRECTORY.concat(m_key);
    } else {
      return DIRECTORY.concat("/").concat(m_key);
    }
  }
}
