// Copyright (c) 2026 Bensalem High School Fightin' Robotic Owls
// https://github.com/FRO5401
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.Utils.FireControl;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShotLUT {
  private InterpolatingTreeMap<Double, ShotData> shotMap;
  private double velocityScalar = 1;
  private double tunableVelocityScalar = 1;

  public ShotLUT(){
    shotMap = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShotData.interpolate());
  }

  public void put(double distance, ShotData shotParms){
    shotMap.put(distance, shotParms);
  }

  public void put(double distance, double rps, double hoodAngle, double tof){
    put(distance, new ShotData(rps, hoodAngle, tof));
  }

  public ShotData getRawShotData(double distance){
    return getInterpolatedShotData(distance);
  }

  public ShotData getScaleredData(double distance){
    var data = getInterpolatedShotData(distance);
    return new ShotData(data.rps() * velocityScalar * tunableVelocityScalar, data.hoodRotations(), data.tof());
  }

  public void clear(){
    shotMap.clear();
  }

  private ShotData getInterpolatedShotData(double distance){
    return shotMap.get(distance) != null ? shotMap.get(distance) : ShotData.ZEROED;
  }

  public void updateVelocityScalar(double scalar){
    tunableVelocityScalar = scalar;
  }
}
