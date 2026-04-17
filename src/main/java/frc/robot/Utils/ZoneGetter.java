// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotDimensionConstants;
import frc.robot.Constants.FieldConstants.CurrentZone;

public class ZoneGetter {

    public static CurrentZone getCurrentZone(Pose2d robotPose) {
        if (!FieldConstants.fieldZone.contains(robotPose.getTranslation())) {
            return CurrentZone.OUTSIDE_BOUNDS;
        }
        if (isTurretInZone(FieldConstants.redZone, robotPose)) {
            return CurrentZone.RED;

        } else if (isTurretInZone(FieldConstants.blueZone, robotPose)) {
            return CurrentZone.BLUE;

        } else {
            if (robotPose.getMeasureY().baseUnitMagnitude() < FieldConstants.HALF_WAY_LINE) {
                return CurrentZone.NUETRAL_RIGHT;
            } else {
                return CurrentZone.NUETRAL_LEFT;
            }

        }
    }

    public static CurrentZone getCurrentZoneSpecific(Pose2d robotPose) {
        if (!FieldConstants.fieldZone.contains(robotPose.getTranslation())) {
            return CurrentZone.OUTSIDE_BOUNDS;
        }
        if (FieldConstants.redZone.contains(robotPose.getTranslation())) {
            return CurrentZone.RED;

        } else if (FieldConstants.redZone.contains(robotPose.getTranslation())) {
            return CurrentZone.RED;

        } else if (FieldConstants.redHub.contains(robotPose.getTranslation())) {
            return CurrentZone.PHASING;

        } else if (FieldConstants.redBump.contains(robotPose.getTranslation())) {
            return CurrentZone.RED_BUMP;

        } else if (FieldConstants.redTrenchBlock.contains(robotPose.getTranslation())) {
            return CurrentZone.PHASING;

        } else if (FieldConstants.redTrench.contains(robotPose.getTranslation())) {
            return CurrentZone.RED_TRENCH;

        } else if (FieldConstants.blueZone.contains(robotPose.getTranslation())) {
            return CurrentZone.BLUE;

        } else if (FieldConstants.blueHub.contains(robotPose.getTranslation())) {
            return CurrentZone.PHASING;

        } else if (FieldConstants.blueBump.contains(robotPose.getTranslation())) {
            return CurrentZone.BLUE_BUMP;

        } else if (FieldConstants.blueTrenchBlock.contains(robotPose.getTranslation())) {
            return CurrentZone.PHASING;

        } else if (FieldConstants.blueTrench.contains(robotPose.getTranslation())) {
            return CurrentZone.BLUE_TRENCH;

        }
        if (robotPose.getMeasureY().baseUnitMagnitude() < FieldConstants.HALF_WAY_LINE) {
            return CurrentZone.NUETRAL_RIGHT;
        } else {
            return CurrentZone.NUETRAL_LEFT;
        }
    }

    public static boolean isShootingZone(Pose2d robotPose) {
        // Prevents simulation without driver station from erroring
        if (DriverStation.getAlliance().isEmpty()) {
            return getCurrentZone(robotPose).equals(CurrentZone.RED);
        }
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return getCurrentZone(robotPose).equals(CurrentZone.BLUE);
        } else {
            return getCurrentZone(robotPose).equals(CurrentZone.RED);
        }
    }

    public static Pose2d getShootingTarget(Pose2d robotPose) {
        if (DriverStation.getAlliance().isEmpty()) {
            return FieldConstants.RED_HUB_TARGET;
        }
        if (DriverStation.getAlliance().get().equals(Alliance.Blue) && isShootingZone(robotPose)) {
            return FieldConstants.BLUE_HUB_TARGET;
        } else if (DriverStation.getAlliance().get().equals(Alliance.Red) && isShootingZone(robotPose)) {
            return FieldConstants.RED_HUB_TARGET;
        }

        if (DriverStation.getAlliance().get().equals(Alliance.Blue) && !isShootingZone(robotPose)) {
            if (getCurrentZone(robotPose).equals(CurrentZone.NUETRAL_LEFT)) {
                return FieldConstants.BLUE_LEFT_PASSING_TARGET;
            } else {
                return FieldConstants.BLUE_RIGHT_PASSING_TARGET;
            }
        } else if (DriverStation.getAlliance().get().equals(Alliance.Red) && !isShootingZone(robotPose)) {
            if (getCurrentZone(robotPose).equals(CurrentZone.NUETRAL_LEFT)) {
                return FieldConstants.RED_LEFT_PASSING_TARGET;
            } else {
                return FieldConstants.RED_RIGHT_PASSING_TARGET;
            }
        }

        return null;
    }

    /** Return true if any corner is within the given zone */
    public static boolean isDrivebaseInZone(Rectangle2d zone, Pose2d robotPose) {
        return zone.contains(robotPose.transformBy(RobotDimensionConstants.BACK_LEFT_CORNER).getTranslation()) ||
                zone.contains(robotPose.transformBy(RobotDimensionConstants.BACK_RIGHT_CORNER).getTranslation()) ||
                zone.contains(robotPose.transformBy(RobotDimensionConstants.FRONT_LEFT_CORNER).getTranslation()) ||
                zone.contains(robotPose.transformBy(RobotDimensionConstants.FRONT_RIGHT_CORNER).getTranslation()) ||
                zone.contains(robotPose.getTranslation());
    }

    public static boolean isTurretInZone(Rectangle2d zone, Pose2d robotPose) {
        return zone.contains(robotPose.transformBy(RobotDimensionConstants.BACK_SIDE).getTranslation()) ||
                zone.contains(robotPose.getTranslation());
    }
}
