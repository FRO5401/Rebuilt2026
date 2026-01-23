package frc.robot.Utils;


import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.MathConstants;

public class MathHelp {
    public static LinearVelocity findFlyWheelVelocity(Transform2d poseDifference){
        //Im gonna slip up the math despite it not being optimal for memory its so much better for readability
        Distance targetDistance = findDistance(poseDifference);
        double numerator = targetDistance.in(Meters)*Math.sqrt(9.8/(2*(Math.tan(MathConstants.LAUNCH_ANGLE.in(Radians))*targetDistance.in(Meters)-MathConstants.HUB_HEIGHT.in(Meters))));
        double denominator = Math.cos(MathConstants.LAUNCH_ANGLE.in(Radians));

        return MetersPerSecond.of(numerator/denominator);
    }

    //Once again splitting up the math, this is the quadatric equation of height displacement formula to find the time of flight
    public static Time findTOF(Transform2d targDistance){
        double a = -4.9;
        double b = findFlyWheelVelocity(targDistance).in(MetersPerSecond) * Math.sin(MathConstants.LAUNCH_ANGLE.in(Radians));
        double c = -MathConstants.HUB_HEIGHT.in(Meters);
        return Seconds.of((-b + Math.sqrt(Math.pow(b, 2)-(4*a*c)))/(2*a));
    }

    //*will find the hypotenuse/resultant of a pose */
    public static Distance findDistance(Transform2d vector){
        return Meters.of(Math.sqrt(Math.pow(vector.getX(), 2)+ Math.pow(vector.getY(), 2)));
    }



}
