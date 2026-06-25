package frc.robot.Utils.FireControl;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotLUT {
  private InterpolatingTreeMap<Double, ShotData> shotMap;
  private boolean hasEntry;

  public ShotLUT(){
    shotMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData.interpolate());

  }

  public void put(double distance, ShotData shotParms){
    shotMap.put(distance, shotParms);
  }

  public void put(double distance, double rps, double hoodAngle, double tof){
    put(distance, new ShotData(rps, hoodAngle, tof));
  }

  public ShotData get(double distance){
    return shotMap.get(distance) != null ? shotMap.get(distance) : ShotData.ZEROED;
  }

  public void clear(){
    shotMap.clear();
  }
}
