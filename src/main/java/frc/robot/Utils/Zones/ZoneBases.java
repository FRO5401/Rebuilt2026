package frc.robot.Utils.Zones;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ZoneBases {
    // basic zones
    public static interface Zone {
        public Trigger contains(Supplier<Pose2d> robotPose);
    }

    // Predictive zones
    public static interface PredictiveZone extends Zone{
        public Trigger willContain(Supplier<Pose2d> robotPose, ChassisSpeeds driveVelocity, Time time);
    }


    public static class ZoneGroup implements Zone{
        protected final Zone[] zones;

        public ZoneGroup(Zone... zones){
            this.zones = zones;
        }

        @Override
        public Trigger contains(Supplier<Pose2d> robotPose) {
            Trigger groupTrigger = new Trigger(()-> false);

            for (Zone zone: zones){
                groupTrigger = groupTrigger.or(zone.contains(robotPose));
            }

            return groupTrigger;
        }

    }

    public static class PredictiveZoneGroup implements PredictiveZone{
        protected final PredictiveZone[] bounds;

        public PredictiveZoneGroup(PredictiveZone... bounds){
            this.bounds = bounds;
        }

        @Override
        public Trigger contains(Supplier<Pose2d> robotPose) {
            Trigger groupTrigger = new Trigger(()-> false);

            for (Zone bound: bounds){
                groupTrigger = groupTrigger.or(bound.contains(robotPose));
            }

            return groupTrigger;
        }

        @Override
        public Trigger willContain(Supplier<Pose2d> robotPose, ChassisSpeeds driveVelocity, Time time) {
            Trigger groupTrigger = new Trigger(()->false);

            for(PredictiveZone bound: bounds){
                groupTrigger = groupTrigger.or(bound.willContain(robotPose, driveVelocity, time));
            }

            return groupTrigger;
        }
        
    }

}
