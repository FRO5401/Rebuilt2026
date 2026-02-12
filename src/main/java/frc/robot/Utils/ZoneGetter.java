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
    if(FieldConstants.blueZoneStart.getX()>robotPose.getX() 
      || FieldConstants.fieldLimits.getX()<robotPose.getX() 
      || FieldConstants.blueZoneStart.getY()>robotPose.getY()
      || FieldConstants.fieldLimits.getY()<robotPose.getY()
      ){
        return CurrentZone.OUTSIDE_BOUNDS;
      }
    if(FieldConstants.redZoneStart.getX() >= robotPose.getX() && robotPose.getX() >= FieldConstants.redZoneEnd.getX()){
      return CurrentZone.RED;

    } else if(FieldConstants.blueZoneStart.getX() <= robotPose.getX() && robotPose.getX() <= FieldConstants.blueZoneEnd.getX()){
      return CurrentZone.BLUE;

    } else {
      return CurrentZone.NUETRAL;
    }
  }

  public static boolean isShootingZone(Pose2d robotPose){
    if(DriverStation.getAlliance().get().equals(Alliance.Blue)){ 
      return getCurrentZone(robotPose).equals(CurrentZone.BLUE);
    } else {
      return getCurrentZone(robotPose).equals(CurrentZone.RED);
    }

  }
}
