package frc.robot.Utils.Tunable;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Pose2d;

public class TunableNumber2 extends TunableBase<Double> implements DoubleSupplier {

  public TunableNumber2(String key){
    super(key);
  }

  public TunableNumber2(String key, Double defaultValue){
    super(key, defaultValue);
  }

  public TunableNumber2(String key, Double defaultValue, boolean disableTuning){
    super(key, defaultValue, disableTuning);
  }

  @Override
  protected void initTunable() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'initTunable'");
  }

  @Override
  public Double getValue() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getValue'");
  }

  @Override
  public double getAsDouble() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAsDouble'");
  }
  
}
