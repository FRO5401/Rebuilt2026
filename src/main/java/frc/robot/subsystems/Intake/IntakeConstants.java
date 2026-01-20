package frc.robot.subsystems.Intake;

public final class IntakeConstants {
    public static final int PIVOT_MASTER_ID = 1;
    public static final int PIVOT_FOLLOWER_ID = 2;
    public static final int INFEED_ID = 3;

    public static final double PIVOT_GEAR_RATIO = 1;
    public static final double INFEED_GEAR_RATIO = 1;
    
    public static final double PIVOT_STATOR_LIMIT = 60;
    public static final double PIVOT_SUPPLY_LIMIT = 120;

    public static final double INFEED_SUPPLY_LIMIT = 60;
    public static final double INFEED_STATOR_LIMIT = 120; 

    public static final boolean PIVOT_MASTER_INVERT = false;
    public static final boolean PIVOT_FOLLOWER_INVERT = true;

    public static final boolean INFEED_INVERT = false;

    public static final double kp = 1;
    public static final double ki = 1;
    public static final double kd = 1;
    public static final double kg = 1;

    public static final double sim_kp = 1;
    public static final double sim_ki = 1;
    public static final double sim_kd = 1;
    public static final double sim_kg = 1;

}
