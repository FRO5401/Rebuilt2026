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
        pivotInputs.velocityRPS = pivotMaster.getVelocity().getValueAsDouble();

        pivotInputs.masterTempCelcius = pivotMaster.getDeviceTemp().getValueAsDouble();
        pivotInputs.masterAppliedVolts = pivotMaster.getMotorVoltage().getValueAsDouble();
        pivotInputs.masterSupplyCurrent = pivotMaster.getSupplyCurrent().getValueAsDouble();
        pivotInputs.masterStatorCurrent = pivotMaster.getStatorCurrent().getValueAsDouble();

        pivotInputs.followerTempCelcius = pivotFollower.getDeviceTemp().getValueAsDouble();
        pivotInputs.followerAppliedVolts = pivotFollower.getMotorVoltage().getValueAsDouble();
        pivotInputs.followerSupplyCurrent = pivotFollower.getSupplyCurrent().getValueAsDouble();
        pivotInputs.followerStatorCurrent = pivotFollower.getStatorCurrent().getValueAsDouble();

        infeedInputs.velocityRPS = infeed.getVelocity().getValueAsDouble();
        infeedInputs.tempCelcius = infeed.getDeviceTemp().getValueAsDouble();
        infeedInputs.appliedVolts = infeed.getMotorVoltage().getValueAsDouble();
        infeedInputs.supplyCurrent = infeed.getSupplyCurrent().getValueAsDouble();
        infeedInputs.statorCurrent = infeed.getStatorCurrent().getValueAsDouble();
    }
    @Override
    public void setPivotPosition(double angle){
        pivotMaster.setControl(pivotPositionRequest.withPosition(angle));
    }

    @Override
    public void setInfeedVelocity(double velocity){
        infeed.set(velocity);
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
