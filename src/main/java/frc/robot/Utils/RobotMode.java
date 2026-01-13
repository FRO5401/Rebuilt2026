package frc.robot.Utils;

import edu.wpi.first.wpilibj.RobotBase;

public class RobotMode {

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static enum Mode{
        REAL,
        SIM
    }
}
