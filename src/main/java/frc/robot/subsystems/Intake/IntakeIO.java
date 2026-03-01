package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class IntakeIOInputs{
        public double pivotAngle = 0;
        public double pivotVelocity = 0;

        public double pivotTemperature = 0;

        public double pivotVoltage = 0;
        public double pivotCurrent = 0;

        public double infeedVelocity = 0;

        public double infeedTemperature = 0;
        public double infeedVoltage = 0;
        public double infeedCurrent = 0;
    }
    
    public default void updateIntakeInputs(IntakeIOInputs intakeInputs){}

    public default void setPivotPosition(double angle){}

    public default void setInfeedVelocity(double percent){}

    public default void stop(){}

    public default void setPivotVoltage(double voltage){}

    public default void setInfeedVoltage(double voltage){}

    public default void setEncoderPosition(double position){}
    
    public default void setPivotPID(double kp, double ki, double kd, double kv, double ks){}
    
} 