package frc.robot.Utils.Zones;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
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

    public class Bounds implements Zone{
        protected final double xMin, xMax, yMin, yMax;

        public Bounds(double xMin, double xMax, double yMin, double yMax){
            this.xMin = xMin;
            this.xMax = xMax;
            this.yMin = yMin;
            this.yMax = yMax;
        }

        public Bounds(Distance xMin, Distance xMax, Distance yMin, Distance yMax){
            this(xMin.in(Meters), xMax.in(Meters), yMin.in(Meters), yMax.in(Meters));
        }

        @Override
        public Trigger contains(Supplier<Pose2d> robotPose) {
            return new Trigger(()-> isWithin(robotPose.get().getTranslation()));
        }

        protected boolean isWithin(Translation2d pose){
            return (xMin <= pose.getX() && pose.getX() <= xMax) && (yMin <= pose.getY() && pose.getY() <= yMax);
        }
        
    }

}
