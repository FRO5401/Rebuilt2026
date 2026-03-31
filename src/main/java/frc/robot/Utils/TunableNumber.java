package frc.robot.utils;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class TunableNumber implements DoubleSupplier {
    private static final String DIRECTORY = "/Tunable";
    private final String key;

    private LoggedNetworkNumber networkNumber;

    private double defaultValue;
    private boolean hasDefault = false; 
    private boolean isTuningDisabled = false; 

    public TunableNumber(String m_key){
        this.key = DIRECTORY + m_key;
    }

    public TunableNumber(String m_key, double m_defaultValue){
        this(m_key);
        initalizeDefault(m_defaultValue);
    }

    public TunableNumber(String m_key, double m_defaultValue, boolean m_disableTuning){
        this(m_key);
        this.isTuningDisabled = m_disableTuning;
        initalizeDefault(m_defaultValue);
    }

    public void disableTuning(boolean m_disable){
        this.isTuningDisabled = m_disable;
    }

    public void initalizeDefault(double m_defaultValue){
        if(!hasDefault){
            this.hasDefault = true;
            this.defaultValue = m_defaultValue;

            if(RobotMode.isTuningMode && !isTuningDisabled){
                //tunableValue = defaultValue;
                networkNumber = new LoggedNetworkNumber(key, defaultValue);
            }
            
        }
    }

    public double get(){
        if(!hasDefault){
            return 0.0;
        } else {
            return (RobotMode.isTuningMode && !isTuningDisabled) ? networkNumber.get() : defaultValue;
        }
    }

    public boolean hasChanged(){

        double currentValue = this.get();
        if(currentValue != defaultValue){
            networkNumber.set(currentValue);
            defaultValue = currentValue;
            return true;
        }
        return false;
    }

    public static boolean hasChanged(TunableNumber... tunables){
        if(Arrays.stream(tunables).anyMatch(tunable -> tunable.hasChanged())){
            return true;
        }
        return false;
    }

    @Override
    public double getAsDouble() {
        return get();
    }
}