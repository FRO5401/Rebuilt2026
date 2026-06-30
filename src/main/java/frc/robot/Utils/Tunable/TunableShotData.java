package frc.robot.Utils.Tunable;

import java.util.function.Supplier;
import edu.wpi.first.networktables.NetworkTable;
import frc.robot.Utils.FireControl.ShotData;

public class TunableShotData extends TunableBase<ShotData> implements Supplier<ShotData>{

  private TunableNumber flywheelRPS;
  private TunableNumber hoodRotations;
  private TunableNumber tof;

  private NetworkTable netTable;

  public TunableShotData(String m_key){
    super(m_key);
  }

  public TunableShotData(String m_key, ShotData m_defaultValue){
    super(m_key, m_defaultValue);
  }

  public TunableShotData(String m_key, ShotData m_defaultValue, boolean m_disableTuning){
    super(m_key, m_defaultValue, m_disableTuning);
  }

  @Override
  protected void initTunable() {
    this.netTable = INSTANCE.getTable(key);
    this.flywheelRPS = new TunableNumber("RPS", defaultValue.rps(), netTable);
    this.hoodRotations = new TunableNumber("HoodRotations", defaultValue.hoodRotations(), netTable);
    this.tof = new TunableNumber("ToF", defaultValue.tof(), netTable);

  }

  @Override
  public ShotData getValue() {
    if(!hasDefault){
      return ShotData.ZEROED;
    } else {
      return isMasterTuningEnabled() ? new ShotData(flywheelRPS.get(), hoodRotations.get(), tof.get()) : defaultValue;
    }
  }

  @Override
  public ShotData get() {
    return getValue();
  }

}
