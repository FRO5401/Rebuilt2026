package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

    @AutoLog
    public static class PivotIOInputs{
        public double angle = 0;
        public double velocityRPS = 0;

        public double masterTempCelcius = 0;

        public double masterAppliedVolts = 0;
        public double masterStatorCurrent = 0;
        public double masterSupplyCurrent = 0; 

        public double followerTempCelcius = 0;

        public double followerAppliedVolts = 0;
        public double followerStatorCurrent = 0;
        public double followerSupplyCurrent = 0;

    }

    @AutoLog
    public static class InfeedIOInputs{
        public double velocityRPS = 0;

        public double tempCelcius = 0;
        public double appliedVolts = 0;
        public double statorCurrent = 0;
        public double supplyCurrent = 0;
    }

    public default void updateIntakeInputs(PivotIOInputs pivotInputs, InfeedIOInputs infeedInputs){}

    public default void setPivotPosition(double angle){}

    public default void setInfeedVelocity(double velocity){}

    public default void stop(){}

    public default void setPivotVoltage(double voltage){}

    public default void setInfeedVoltage(double voltage){}

    public default void setIntake(double pivotAngle, double infeedVelocity){
        setPivotPosition(pivotAngle);
        setInfeedVelocity(infeedVelocity);
    }

    public default void setPivotPID(double kp, double ki, double kd){}
    
} 