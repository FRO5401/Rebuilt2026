package frc.robot.Utils.Tunable;

import java.util.function.Supplier;
import frc.robot.Utils.FireControl.ShotData;

public class TunableShotData extends TunableBase<ShotData> implements Supplier<ShotData>{

  private TunableNumber flywheelRPS;
  private TunableNumber hoodRotations;
  private TunableNumber tof;

  public TunableShotData(String key){
    super(key);
  }

  public TunableShotData(String key, ShotData defaultValue){
    super(key, defaultValue);
  }

  public TunableShotData(String key, ShotData defaultValue, boolean disableTuning){
    super(key, defaultValue, disableTuning);
  }

  @Override
  protected void initTunable() {
    flywheelRPS = new TunableNumber(key.concat("RPS"), defaultValue.rps());
    hoodRotations = new TunableNumber(key.concat("HoodRotations"), defaultValue.hoodRotations());
    tof = new TunableNumber(key.concat("ToF"), defaultValue.tof());
  }

  @Override
  public ShotData getValue() {
    if(!hasDefault){
      return ShotData.ZEROED;
    } else {
      return new ShotData(flywheelRPS.get(), hoodRotations.get(), tof.get());
    }
  }

  @Override
  public ShotData get() {
    return getValue();
  }

}
