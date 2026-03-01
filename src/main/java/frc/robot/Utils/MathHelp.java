package frc.robot.Utils;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import frc.robot.Constants.MathConstants;
import frc.robot.Constants.ShooterConstants;

public class MathHelp {
    public static TunableNumber RPM = new TunableNumber("/Shooter/FlyWheel", 0);

    public static LinearVelocity findFlyWheelVelocity(Transform2d poseDifference) {
        // Im gonna slip up the math despite it not being optimal for memory its so much
        // better for readability
        LinearVelocity velocity;

        if (Robot.isSimulation()) {
            Distance targetDistance = findDistance(poseDifference);
            double numerator = targetDistance.in(Meters)
                    * Math.sqrt(9.8 / (2 * (Math.tan(MathConstants.LAUNCH_ANGLE.in(Radians)) * targetDistance.in(Meters)
                            - MathConstants.HUB_HEIGHT.in(Meters))));
            double denominator = Math.cos(MathConstants.LAUNCH_ANGLE.in(Radians));

            velocity = MetersPerSecond.of((numerator / denominator) / MathConstants.FLYWHEEL_EFFICIENCY);
        } else {
            velocity = MetersPerSecond.of(ShooterConstants.TREE_MAP.get(findDistance(poseDifference).in(Meters)));
        }

        Logger.recordOutput("MathHelp/Flywheel Velocity", velocity.in(MetersPerSecond));

        return velocity;
    }

    public static AngularVelocity findFlyWheelRPM(LinearVelocity flywheelVelocity) {
        // 60 is for the seconds to minute, 3.82
        if(Robot.isSimulation()){
        Logger.recordOutput("MathHelp/Flywheel RPM",
                (flywheelVelocity.in(MetersPerSecond)) / (Math.PI * MathConstants.FLY_WHEEL_DIAMETER.in(Meters)) * 60);
        return RotationsPerSecond
                .of((flywheelVelocity.in(MetersPerSecond)) / (Math.PI * MathConstants.FLY_WHEEL_DIAMETER.in(Meters)));
        } else {return RotationsPerSecond.of(RPM.get());
    }
}

    // Once again splitting up the math, this is the quadatric equation of height
    // displacement formula to find the time of flight
    public static Time findTOF(Transform2d targDistance) {
        double a = -4.9;
        double b =  findFlyWheelVelocity(targDistance).baseUnitMagnitude()
                * Math.sin(MathConstants.LAUNCH_ANGLE.in(Radians));
        double c = -MathConstants.HUB_HEIGHT.in(Meters);
        return Seconds.of((-b - Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a));
    }

    // *will find the hypotenuse/resultant of a pose */
    public static Distance findDistance(Transform2d vector) {
        if (vector != null) {
            Logger.recordOutput("Distance", Math.sqrt(Math.pow(vector.getX(), 2) + Math.pow(vector.getY(), 2)));
            return Meters.of(Math.sqrt(Math.pow(vector.getX(), 2) + Math.pow(vector.getY(), 2)));
        } else {
            return Meters.of(0);
        }

    }

    public static double RPMtoVelocity(double RPM){
        Logger.recordOutput("MathHelp/Ball Velocity", .43*(RPM*MathConstants.FLY_WHEEL_DIAMETER.baseUnitMagnitude()*Math.PI));
        return .43*(RPM*MathConstants.FLY_WHEEL_DIAMETER.baseUnitMagnitude()*Math.PI);
    }

}
