package frc.robot.Utils.Tunable;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import frc.robot.Utils.RobotMode;

public abstract class TunableBase<V> {
  protected static final String DIRECTORY = "/Tunable";
  protected final String key;

  protected V defaultValue;
  protected boolean hasDefault = false;

  protected Map<Integer, V> lastValueMap = new HashMap<>();

  protected boolean isTuningDisabled = false;
  protected boolean masterTuningEnabled = RobotMode.isTuningMode ? !isTuningDisabled : false;

  protected TunableBase(String key){
    this.key = DIRECTORY.concat(key);
  }

  protected TunableBase(String key, V defaultValue){
    this(key);
    setDefaultValue(defaultValue);
  }

  protected TunableBase(String key, V defaultValue, boolean disableTuning){
    this(key);
    setDefaultValue(defaultValue);
    this.isTuningDisabled = disableTuning;
  }

  public abstract V getValue();

  protected abstract void initTunable();


  public void setDefaultValue(V defaultValue){
    if(!hasDefault){
      this.hasDefault = true;
      this.defaultValue = defaultValue;

      if(masterTuningEnabled) initTunable();
    }

  }

  public boolean hasChanged(int id) {
    V currentValue = getValue();
    V lastValue = lastValueMap.get(id);
    if (lastValue != null && !lastValue.equals(currentValue)) {
      lastValueMap.put(id, currentValue);
      return true;
    }

    return false;
  }

  public static boolean hasChanged(int id, TunableBase... tunables) {
    if (Arrays.stream(tunables).anyMatch(tunable -> tunable.hasChanged(id))) {
      return true;
    }
    return false;
  }

}
