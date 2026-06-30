// Copyright (c) 2026 Bensalem High School Fightin' Robotic Owls
// https://github.com/FRO5401
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// Contributors:
//  2026 Soccertoad Ben Reinert
//  2026 Ace-h121 Ace Hathaway

package frc.robot.Utils.Tunable;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Utils.RobotMode;

// Java Docs as of 6/30/26 were written by AI
// All logic and code was created without AI


/**
 * A tunable boolean value backed by NetworkTables.
 *
 * <p>
 * When tuning mode is enabled, the value is published to NetworkTables and can be modified live
 * through dashboards such as AdvantageScope, Shuffleboard, or Elastic. When tuning mode is
 * disabled, this class simply returns the provided default value with no NetworkTables overhead.
 *
 * <p>
 * This class also implements {@link BooleanSupplier}, allowing it to be passed directly into
 * commands, controllers, and other WPILib APIs that accept a {@code BooleanSupplier}.
 */
public class TunableBoolean implements BooleanSupplier {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    private static final String DIRECTORY = "/Tunable";

    private String key;

    private static final NetworkTable DATA_TABLE = INSTANCE.getTable(DIRECTORY);
    private BooleanSubscriber sub;
    private BooleanPublisher pub;

    private boolean defaultValue;

    private boolean hasDefault = false;
    private boolean isTuningDisabled = false;

    /**
   * Returns whether tuning is currently active.
   *
   * <p>Tuning is active only when robot mode allows it and this tunable is not disabled.
   */
    private boolean isTuningEnabled(){
        return RobotMode.isTuningMode ? !isTuningDisabled : false;
    }

    /**
     * Creates a tunable boolean without a default value.
     *
     * <p>
     * {@link #initalizeDefault(boolean)} must be called before the value can be used.
     *
     * @param m_key NetworkTables key for this value.
     */
    public TunableBoolean(String m_key) {
        this.key = m_key;
    }

    /**
     * Creates a tunable boolean with the specified default value.
     *
     * @param m_key NetworkTables key.
     * @param m_defaultValue Default value when tuning is disabled.
     */
    public TunableBoolean(String m_key, boolean m_defaultValue) {
        this(m_key);
        initializeDefault(m_defaultValue);
    }

    /**
     * Creates a tunable boolean with an option to disable tuning.
     *
     * @param m_key NetworkTables key.
     * @param m_defaultValue Default value.
     * @param m_disableTuning If true, this value will never be tunable even when robot tuning mode
     *        is enabled.
     */
    public TunableBoolean(String m_key, boolean m_defaultValue, boolean m_disableTuning) {
        this(m_key);
        this.isTuningDisabled = m_disableTuning;
        initializeDefault(m_defaultValue);
    }

    /**
     * Creates a tunable boolean using a custom NetworkTable.
     *
     * @param m_key NetworkTables key.
     * @param m_defaultValue Default value.
     * @param netTable NetworkTable to publish and subscribe through.
     */
    public TunableBoolean(String m_key, boolean m_defaultValue, NetworkTable netTable) {
        this(m_key);
        initializeDefault(m_defaultValue, netTable);
    }

    /**
     * Initializes the default value and creates the NetworkTables publisher and subscriber if
     * tuning is enabled.
     *
     * <p>
     * This method only has an effect the first time it is called.
     *
     * @param m_defaultValue Initial value of the tunable boolean.
     */
    public void initializeDefault(boolean m_defaultValue) {
        if (!hasDefault) {
            this.hasDefault = true;
            this.defaultValue = m_defaultValue;

            if (isTuningEnabled()) {
                pub = DATA_TABLE.getBooleanTopic(key).publish();
                pub.set(defaultValue);

                sub = DATA_TABLE.getBooleanTopic(key).subscribe(defaultValue);
            }

        }
    }

    /**
     * Initializes the default value using a custom NetworkTable.
     *
     * <p>
     * This method only has an effect the first time it is called.
     *
     * @param m_defaultValue Initial value of the tunable boolean.
     * @param netTable NetworkTable used for publishing and subscribing.
     */
    public void initializeDefault(boolean m_defaultValue, NetworkTable netTable) {
        if (!hasDefault) {
            this.hasDefault = true;
            this.defaultValue = m_defaultValue;

            if (isTuningEnabled()) {
                pub = netTable.getBooleanTopic(key).publish();
                pub.set(defaultValue);

                sub = netTable.getBooleanTopic(key).subscribe(defaultValue);
            }

        }
    }

    /**
     * Returns the current value of this tunable boolean.
     *
     * <p>
     * If tuning is enabled, the value is read from NetworkTables. Otherwise, the stored default
     * value is returned.
     *
     * @return Current value.
     */
    public boolean get() {
        if (!hasDefault) {
            return false;
        } else {
            return isTuningEnabled() ? sub.get() : defaultValue;
        }
    }

    /**
     * Determines whether the value has changed since the previous call.
     *
     * <p>
     * If a new value has been received from NetworkTables, the stored value is updated and this
     * method returns {@code true}.
     *
     * @return {@code true} if the value changed, otherwise {@code false}.
     */
    public boolean hasChanged() {
        if (!isTuningEnabled()) return false;

        boolean currentValue = this.get();
        if (currentValue != defaultValue) {
            pub.set(currentValue);
            defaultValue = currentValue;
            return true;
        }
        return false;
    }

    /**
     * Determines whether any of the supplied tunable booleans have changed.
     *
     * @param tunables Tunable booleans to check.
     * @return {@code true} if any value has changed.
     */
    public static boolean hasChanged(TunableBoolean... tunables) {
        if (Arrays.stream(tunables).anyMatch(tunable -> tunable.hasChanged())) {
            return true;
        }
        return false;
    }

    /**
     * Returns the current value as required by the {@link BooleanSupplier} interface.
     *
     * @return Current value.
     */
    @Override
    public boolean getAsBoolean() {
        return get();
    }

    /**
     * Closes the underlying NetworkTables publisher and subscriber.
     *
     * <p>
     * This should be called when the tunable boolean is no longer needed to release NetworkTables
     * resources.
     */
    public void close() {
        if (pub != null) pub.close();
        if (sub != null) sub.close();;
    }

}

