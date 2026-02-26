package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO { 
    @AutoLog
    public class ShooterIOInputs {
        public double velocity = 0;
        public double voltage;
        public double current = 0;
    }

    public default void updateInputs(ShooterIOInputs inputs){};

    public default void applyVoltage(double voltage){};

    public default void setVelocity(double velocity, ShooterIOInputs inputs){};

    public default void applyPID(double P, double I, double D, double S, double V){};

    public default void stop(){};

}
