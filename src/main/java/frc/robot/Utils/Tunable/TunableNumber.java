package frc.robot.Utils.Tunable;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Utils.RobotMode;

public class TunableNumber implements DoubleSupplier {
    private static final NetworkTableInstance INSTANCE = NetworkTableInstance.getDefault();
    private static final String DIRECTORY = "/Tunable";

    private String key;

    private static final NetworkTable DATA_TABLE = INSTANCE.getTable(DIRECTORY);
    private DoubleSubscriber sub;
    private DoublePublisher pub;

    private double defaultValue;
    private boolean hasDefault = false;
    private boolean isTuningDisabled = false;
    private boolean masterTuningEnabled = RobotMode.isTuningMode ? !isTuningDisabled : false;

    public TunableNumber(String m_key) {
        this.key = m_key;
    }

    public TunableNumber(String m_key, double m_defaultValue) {
        this(m_key);
        initializeDefault(m_defaultValue);
    }

    public TunableNumber(String m_key, double m_defaultValue, boolean m_disableTuning) {
        this(m_key);
        this.isTuningDisabled = m_disableTuning;
        initializeDefault(m_defaultValue);
    }

    public TunableNumber(String m_key, double m_defaultValue, NetworkTable netTable) {
        this(m_key);
        initializeDefault(m_defaultValue, netTable);
    }

    public void initializeDefault(double m_defaultValue) {
        if (!hasDefault) {
            this.hasDefault = true;
            this.defaultValue = m_defaultValue;

            if (masterTuningEnabled) {
                pub = DATA_TABLE.getDoubleTopic(key).publish();
                pub.set(defaultValue);

                sub = DATA_TABLE.getDoubleTopic(key).subscribe(defaultValue);
            }

        }
    }

    public void initializeDefault(double m_defaultValue, NetworkTable netTable) {
        if (!hasDefault) {
            this.hasDefault = true;
            this.defaultValue = m_defaultValue;

            if (masterTuningEnabled) {
                pub = netTable.getDoubleTopic(key).publish();
                pub.set(defaultValue);

                sub = netTable.getDoubleTopic(key).subscribe(defaultValue);
            }

        }
    }

    public double get() {
        if (!hasDefault) {
            return 0.0;
        } else {
            return masterTuningEnabled ? sub.get() : defaultValue;
        }
    }

    public boolean hasChanged() {
        if (!masterTuningEnabled) return false;

        double currentValue = this.get();
        if (currentValue != defaultValue) {
            pub.set(currentValue);
            defaultValue = currentValue;
            return true;
        }
        return false;
    }

    public static boolean hasChanged(TunableNumber... tunables) {
        if (Arrays.stream(tunables).anyMatch(tunable -> tunable.hasChanged())) {
            return true;
        }
        return false;
    }

    @Override
    public double getAsDouble() {
        return get();
    }

    public void close(){
        if(pub != null) pub.close();
        if(sub != null) sub.close();
    }
}