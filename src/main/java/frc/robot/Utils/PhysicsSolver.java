package frc.robot.Utils;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.measure.Time;
import frc.robot.Constants.MathConstants;
import frc.robot.Utils.RobotMode.Mode;

public class PhysicsSolver {

    public static Time solveTimeOfFlight(Transform2d targDistance) {

        if (RobotMode.currentMode == Mode.SIM) {
            return MathHelp.findTOF(targDistance);
        }

        double launchVelocity = MathHelp.findFlyWheelVelocity(targDistance).in(MetersPerSecond)
                * MathConstants.FLYWHEEL_EFFICIENCY;

        double area = Math.PI * Math.pow(MathConstants.BALL_DIAMETER.in(Meters) / 2.0, 2);
        double mass = MathConstants.BALL_MASS.in(Kilograms);

        double angle = MathConstants.LAUNCH_ANGLE.in(Radians);

        double vx = launchVelocity * Math.cos(angle);
        double vy = launchVelocity * Math.sin(angle);

        double x = 0;
        double y = 0;

        double dt = 0.001;
        double time = 0;

        double dragFactor = 0.5 * MathConstants.RHO * MathConstants.CD * area / mass;

        while (x < MathHelp.findDistance(targDistance).in(Meters) && y >= -1) {

            double v = Math.sqrt(vx * vx + vy * vy);

            double ax = -dragFactor * v * vx;
            double ay = -dragFactor * v * vy - MathConstants.GRAVITY.in(MetersPerSecondPerSecond);

            vx += ax * dt;
            vy += ay * dt;

            x += vx * dt;
            y += vy * dt;

            time += dt;
        }

        return Seconds.of(time);
    }


}