package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants.IndexerConstants;

public class IndexerIOTalon implements IndexerIO {
    
    private TalonFX spindexer;
    private TalonFX kicker;


    public IndexerIOTalon() {
        spindexer = new TalonFX(IndexerConstants.SPINDEXER_ID);
        kicker = new TalonFX(IndexerConstants.KICKER_ID);

        spindexer.getConfigurator().apply(IndexerConstants.SPINDEXER_CONFIG);
        kicker.getConfigurator().apply(IndexerConstants.KICKER_CONFIG);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.kickerCurrent = kicker.getSupplyCurrent().getValueAsDouble();
        inputs.kickerVelocity = kicker.getVelocity().getValueAsDouble();
        inputs.kickerVoltage = kicker.getMotorVoltage().getValueAsDouble();
        inputs.kickerTemperature = kicker.getDeviceTemp().getValueAsDouble();

        inputs.spindexerCurrent = spindexer.getSupplyCurrent().getValueAsDouble();
        inputs.spindexerVelocity = spindexer.getVelocity().getValueAsDouble();
        inputs.spindexerVoltage = spindexer.getMotorVoltage().getValueAsDouble();
        inputs.spindexerTemperature = spindexer.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void stop() {
        spindexer.set(0);
        kicker.set(0);
    }

    @Override
    public void setSpindexerVoltage(double voltage) {
        spindexer.setVoltage(voltage);
    }

    @Override
    public void setSpindexerVelocity(double velocity) {
        spindexer.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void setSpindexerPercent(double percent) {
        spindexer.set((percent));
    }

    @Override
    public void setKickerVoltage(double voltage) {
        kicker.setVoltage(voltage);
    }

    @Override
    public void setKickerVelocity(double velocity) {
        kicker.setControl(new VelocityVoltage(velocity));
    }

    @Override
    public void setKickerPercent(double percent) {
        kicker.set(percent);
    }
}
