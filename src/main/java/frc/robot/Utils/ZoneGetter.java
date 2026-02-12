// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.CurrentZone;

/** Add your docs here. */
public class ZoneGetter {
    
  public static CurrentZone getCurrentZone(Pose2d robotPose){
    if(!FieldConstants.fieldZone.contains(robotPose.getTranslation())){
        return CurrentZone.OUTSIDE_BOUNDS;
    }
    if(FieldConstants.redZone.contains(robotPose.getTranslation())){
      return CurrentZone.RED;

    } else if(FieldConstants.blueZone.contains(robotPose.getTranslation())){
      return CurrentZone.BLUE;

    } else {
      return CurrentZone.NUETRAL;
    }
  }

  public static boolean isShootingZone(Pose2d robotPose){
    // Prevents simulation without driver station from erroring 
    if(DriverStation.getAlliance().isEmpty()){
      return getCurrentZone(robotPose).equals(CurrentZone.RED);
    } 
    if(DriverStation.getAlliance().get().equals(Alliance.Blue)){ 
      return getCurrentZone(robotPose).equals(CurrentZone.BLUE);
    } else {
      return getCurrentZone(robotPose).equals(CurrentZone.RED);
    }

  }
}
