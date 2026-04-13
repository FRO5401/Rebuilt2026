package frc.robot.Utils.Zones;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldZones;
import frc.robot.Utils.MathHelp;
import frc.robot.Utils.Zones.ZoneBases.Zone;

public class Triangle {
    public static class TriangleBound implements Zone{

        protected final double x1, x2, x3, y1, y2, y3, area;

        public TriangleBound(double x1, double y1, double x2, double y2, double x3, double y3){
            this.x1 = x1;
            this.x2 = x2; 
            this.x3 = x3;
            this.y1 = y1;
            this.y2 = y2;
            this.y3 = y3;
            this.area = getArea();
        }

        public TriangleBound(Translation2d pointA, Translation2d pointB, Translation2d pointC){
            this(pointA.getX(), pointA.getY(), pointB.getX(), pointB.getY(), pointC.getX(), pointC.getY());
        }

        public TriangleBound(Distance x1, Distance y1, Distance x2, Distance y2, Distance x3, Distance y3){
            this(x1.in(Meters), x2.in(Meters), x3.in(Meters), y1.in(Meters), y2.in(Meters), y3.in(Meters)); 
        }

        protected double getArea(){
            return Math.abs((x1*(y2-y3) + x2*(y3-y1)+x3*(y1-y2))/2.0);
        } 

        protected double getArea(double x1, double y1, double x2, double y2, double x3, double y3){
            return Math.abs((x1*(y2-y3) + x2*(y3-y1)+x3*(y1-y2))/2.0);
        }

        protected boolean isWithin(Translation2d pose){
            double A1 = getArea(pose.getX(), pose.getY(), x2, y2, x3, y3);
            double A2 = getArea(x1, y1, pose.getX(), pose.getY(), x3, y3);
            double A3 = getArea(x1, y1, x2, y2, pose.getX(), pose.getY());
            return MathHelp.epsilonEquals(area, A1 + A2 + A3);
        }
        
        @Override
        public Trigger contains(Supplier<Pose2d> robotPose) {
            return new Trigger(()->isWithin(robotPose.get().getTranslation()));
        }
        
        public Translation2d[] getCorners(){
            return new Translation2d[] {
                new Translation2d(x1, y1), 
                new Translation2d(x2, y2), 
                new Translation2d(x3, y3),
                new Translation2d(x1, y1)
            };
        }

        public TriangleBound getMirrorBound(){
            return new TriangleBound(FieldZones.FIELD_LENGTH.in(Meters) - x3, FieldZones.FIELD_WIDTH.in(Meters) - y3, FieldZones.FIELD_LENGTH.in(Meters) - x2, FieldZones.FIELD_WIDTH.in(Meters) - y2, FieldZones.FIELD_LENGTH.in(Meters) - x1, FieldZones.FIELD_WIDTH.in(Meters) - y1);
        }
    }

}
