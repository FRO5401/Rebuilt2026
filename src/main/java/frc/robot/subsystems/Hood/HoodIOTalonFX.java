package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class HoodIOTalonFX implements HoodIO{

    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private final TalonFX hood = new TalonFX(0);
    
    public HoodIOTalonFX(){
        hood.getConfigurator().apply(null)
    }
    
    @Override
    public void updateInputs(HoodIOInputs inputs) {}
    
    @Override
    public void setPosition(double position) {}
    
    @Override
    public void setVoltage(double voltage) {}
    
    @Override
    public void stop() {}
    
    @Override
    public void setPID(double p, double i, double d, double ff) {}

}
