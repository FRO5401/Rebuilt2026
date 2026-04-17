package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class PivotIOInputs {
        public double angle = 0;
        public double velocity = 0;

        public double temperature = 0;

        public double voltage = 0;
        public double current = 0;

        public double followerTemperature = 0;

        public double followerVoltage = 0;
        public double followerCurrent = 0;
    }

    @AutoLog
    public static class InfeedIOInputs {
        public double velocity = 0;

        public double temperature = 0;
        public double voltage = 0;
        public double current = 0;
    }

    public default void updateIntakeInputs(PivotIOInputs pivotInputs, InfeedIOInputs infeedInputs) {}

    public default void setPivotPosition(double angle) {}

    public default void setInfeedVelocity(double percent) {}

    public default void stop() {}

    public default void setPivotVoltage(double voltage) {}

    public default void setInfeedVoltage(double voltage) {}

    public default void setPivotPID(double kp, double ki, double kd, double kv, double ks) {}

}