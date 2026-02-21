package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    @AutoLog
    public static class IndexerIOInputs {
        double spindexerVelocity = 0;
        double spindexerTemperature = 0;
        double spindexerVoltage = 0;
        double spindexerCurrent = 0;

        double kickerVelocity = 0;
        double kickerTemperature = 0;
        double kickerVoltage = 0;
        double kickerCurrent = 0;
    }

    default void updateInputs(IndexerIOInputs inputs) {}

    default void setSpindexerVoltage(double voltage) {}

    default void setSpindexerVelocity(double velocity) {}

    default void setSpindexerPercent(double percent) {}

    default void setKickerVoltage(double voltage) {}

    default void setKickerVelocity(double velocity) {}

    default void setKickerPercent(double percent) {}

    default void stop() {}
}