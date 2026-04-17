package frc.robot.subsystems.Turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    @AutoLog
    public class TurretIOInputs{
        public double position = 0;
        public double velocity = 0;
        public double temperature = 0;
        public double voltage;
        public double current = 0;
        public double applied = 0;
    }

    public default void updateInputs(TurretIOInputs inputs){};

    public default void applyVoltage(double voltage){};

    public default void stop(){};

    public default void applyDutyCycle(double percent){};

    public default void setPosition(double position, double velocity){};

    public default void setPID(double p, double i, double d, double kS, double kV){}
    
}
