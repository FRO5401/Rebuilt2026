package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {

  @AutoLog
  public static class HoodIOInputs {
    public double motorPosition = 0;
    public double motorVelocity = 0;
    public double temperature = 0;
    public double voltage;
    public double current = 0;
    public double applied = 0;

    public double encoderPosition = 0;
    public double encoderVelocity = 0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double voltage) {}

  public default void stop() {}

  public default void setPID(double p, double i, double d, double ff) {}
}
