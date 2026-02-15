package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

public class IntakeIOTalonFX implements IntakeIO{

    private final TalonFX infeed = new TalonFX(0);
    private final TalonFX pivotMaster = new TalonFX(0);
    private final TalonFX pivotFollower = new TalonFX(0);

    private final TalonFXConfiguration infeedConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration pivotConfig = new TalonFXConfiguration();

    private final PositionVoltage pivotPositionRequest = new PositionVoltage(0.0);

    private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);

    public IntakeIOTalonFX(){
        pivotFollower.setControl(new Follower(pivotMaster.getDeviceID(), MotorAlignmentValue.Aligned));

        infeedConfig.Feedback.SensorToMechanismRatio = IntakeConstants.INFEED_GEAR_RATIO;
        infeedConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.INFEED_STATOR_LIMIT;
        infeedConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.INFEED_SUPPLY_LIMIT;

        pivotConfig.Feedback.SensorToMechanismRatio = IntakeConstants.PIVOT_GEAR_RATIO;
        pivotConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.PIVOT_STATOR_LIMIT;
        pivotConfig.CurrentLimits.SupplyCurrentLimit = IntakeConstants.PIVOT_SUPPLY_LIMIT;

        infeed.getConfigurator().apply(infeedConfig);
        pivotMaster.getConfigurator().apply(pivotConfig);
        pivotFollower.getConfigurator().apply(pivotConfig);

    }

    public void updateIntakeInputs(PivotIOInputs pivotInputs, InfeedIOInputs infeedInputs){
        pivotInputs.angle = pivotMaster.getPosition().getValueAsDouble();
        pivotInputs.velocity = pivotMaster.getVelocity().getValueAsDouble();

        pivotInputs.temperature = pivotMaster.getDeviceTemp().getValueAsDouble();
        pivotInputs.voltage = pivotMaster.getMotorVoltage().getValueAsDouble();
        pivotInputs.current = pivotMaster.getSupplyCurrent().getValueAsDouble();

        pivotInputs.followerTemperature = pivotFollower.getDeviceTemp().getValueAsDouble();
        pivotInputs.followerVoltage = pivotFollower.getMotorVoltage().getValueAsDouble();
        pivotInputs.followerCurrent = pivotFollower.getSupplyCurrent().getValueAsDouble();

        infeedInputs.velocity = infeed.getVelocity().getValueAsDouble();
        infeedInputs.temperature = infeed.getDeviceTemp().getValueAsDouble();
        infeedInputs.voltage = infeed.getMotorVoltage().getValueAsDouble();
        infeedInputs.current = infeed.getSupplyCurrent().getValueAsDouble();
    }
    
    @Override
    public void setPivotPosition(double angle){
        pivotMaster.setControl(pivotPositionRequest.withPosition(angle));
    }

    @Override
    public void setInfeedVelocity(double percent){
        infeed.set(percent);
    }

    @Override
    public void stop(){
        infeed.stopMotor();
        pivotMaster.stopMotor();
    }

    @Override
    public void setPivotVoltage(double voltage){
        pivotMaster.setControl(pivotVoltageRequest.withOutput(voltage));
    }
    
    @Override
    public void setInfeedVoltage(double voltage){
        infeed.setVoltage(voltage);
    }
    
}
