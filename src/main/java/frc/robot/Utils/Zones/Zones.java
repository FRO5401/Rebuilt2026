package frc.robot.Utils.Zones;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Zones {
    // basic zones
    public static interface Zone {
        public Trigger contains(Supplier<Pose2d> robotPose);
    }

    // Predictive zones
    public static interface PredictiveZone {
        public Trigger willContain(Supplier<Pose2d> robotPose, ChassisSpeeds driveVelocity, Time time);
    }

    public static class RectangleBounds implements Zone{
        protected final double xMin, xMax, yMin, yMax;

        public RectangleBounds(double xMin, double xMax, double yMin, double yMax){
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
        }

        public RectangleBounds(Distance xMin, Distance xMax, Distance yMin, Distance yMax){
            this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
        }

        public RectangleBounds(Translation2d cornerA, Translation2d cornerB){
            this(cornerA.getX(), cornerB.getX(), cornerA.getY(), cornerB.getY());
        }

        @Override
        public Trigger contains(Supplier<Pose2d> robotPose) {
            return new Trigger(()-> isWithin(robotPose.get().getTranslation()));
        }

        protected boolean isWithin(Translation2d pose){
            return isWithinX(pose) && isWithinY(pose);
        }

        protected boolean isWithinX(Translation2d pose){
            return xMin <= pose.getX() && pose.getX() <= xMax;
        }

        protected boolean isWithinY(Translation2d pose){
            return (yMin <= pose.getY() && pose.getY() <= yMax);
        }
        
    }

    public static class PredictiveRectangleX extends RectangleBounds implements PredictiveZone{
        
        public PredictiveRectangleX(double xMin, double xMax, double yMin, double yMax){
            super(xMin, xMax, yMin, yMax);
        }

        public PredictiveRectangleX(RectangleBounds limits){
            super(limits.xMin, limits.xMax, limits.yMin, limits.yMax);
        }

        public PredictiveRectangleX(Distance xMin, Distance xMax, Distance yMin, Distance yMax){
            super(xMin, xMax, yMin, yMax);
        }

        public PredictiveRectangleX(Translation2d cornerA, Translation2d cornerB){
            super(cornerA, cornerB);
        }

        @Override
        public Trigger willContain(Supplier<Pose2d> robotPose, ChassisSpeeds driveVelocity, Time time) {
            return new Trigger(()-> willBeWithin(robotPose.get().getTranslation(), driveVelocity, time));
        }

        protected boolean willBeWithin(Translation2d robotPose, ChassisSpeeds velocity, Time time){
            return isWithinY(robotPose) && (checkFutureX(robotPose, velocity, time) || isWithinX(robotPose));
        }
        
        protected boolean checkFutureX(Translation2d robotPose, ChassisSpeeds velocity, Time time){
            return (robotPose.getX() < xMin && robotPose.getX() + velocity.vxMetersPerSecond * time.in(Seconds)>xMin) 
            || (robotPose.getX()> xMax && robotPose.getX() + velocity.vxMetersPerSecond * time.in(Seconds)<xMax);
        }
        
    }

}
