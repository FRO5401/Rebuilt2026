package frc.robot.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

//Class used for early testing to fake robot positions on the field
public  class FakePosition {
    Pose2d pose = new Pose2d(0, 0, new Rotation2d());
    
    public void changePose(Pose2d newpose){
        pose = newpose;
    }

    public Pose2d getPose(){
        return pose;
    }
}
