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
  protected boolean masterTuningEnabled = RobotMode.isTuningMode ? !isTuningDisabled : false;

  /**
   * Creates a tunable value with the given key.
   *
   * @param key Name of the tunable entry relative to the tunable directory.
   */
  protected TunableBase(String key) {
    this.key = DIRECTORY + "/" + key;
  }

  /**
   * Creates a tunable value with the specified default value.
   *
   * @param key Name of the tunable entry.
   * @param defaultValue Default value when tuning is disabled.
   */
  protected TunableBase(String key, V defaultValue) {
    this(key);
    setDefaultValue(defaultValue);
  }

  /**
   * Creates a tunable value with an option to disable tuning.
   *
   * @param key Name of the tunable entry.
   * @param defaultValue Default value.
   * @param disableTuning If true, this value will never be tunable.
   */
  protected TunableBase(String key, V defaultValue, boolean disableTuning) {
    this(key);
    setDefaultValue(defaultValue);
    this.isTuningDisabled = disableTuning;
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
   * @param defaultValue Default value to store.
   */
  public void setDefaultValue(V defaultValue) {
    if (!hasDefault) {
      this.hasDefault = true;
      this.defaultValue = defaultValue;

      if (masterTuningEnabled)
        initTunable();
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
    if (!masterTuningEnabled)
      return false;

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

}
