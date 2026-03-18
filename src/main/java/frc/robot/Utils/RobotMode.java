package frc.robot.Utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

public class RobotMode {

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode{
        REAL,
        SIM,
        REPLAY
    }

    //should tuning be on when not connected to FMS
    public static final boolean isTuningOff = true;
    public static final boolean isTuningMode = !(DriverStation.isFMSAttached() || isTuningOff);

}
