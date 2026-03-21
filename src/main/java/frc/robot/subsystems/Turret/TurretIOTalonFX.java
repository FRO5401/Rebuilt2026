// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants.TurretConstants;




/** Add your docs here. */
public class TurretIOTalonFX implements TurretIO {

    VoltageOut voltageRequest = new VoltageOut(0.0);
    PositionVoltage positionRequest = new PositionVoltage(0);

    TalonFX turretMotor = new TalonFX(TurretConstants.CAN_ID);




    public TurretIOTalonFX() {
        turretMotor.getConfigurator().apply(TurretConstants.CONFIG);
        TurretConstants.encoder.setPosition(0.485*TurretConstants.GEAR_RATIO);

    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.voltage = turretMotor.getMotorVoltage().getValueAsDouble();
        inputs.current = turretMotor.getSupplyCurrent().getValueAsDouble();
        inputs.position = turretMotor.getPosition().getValueAsDouble();
        inputs.velocity = turretMotor.getVelocity().getValueAsDouble();
        inputs.applied = turretMotor.get();
    }

    @Override
    public void applyVoltage(double voltage) {
        turretMotor.setControl(voltageRequest.withOutput(voltage));

    }

    @Override
    public void stop() {
        turretMotor.setControl(voltageRequest.withOutput(0.0));

    }

    @Override 
    public void applyDutyCycle(double percent){
        turretMotor.set(percent);
    }

    @Override 
    public void setPosition(double position){
        turretMotor.setControl(positionRequest.withPosition(position));
    }

    @Override
    public void setPID(double p, double i, double d, double kS, double kV){
        TurretConstants.CLOSED_LOOP.kP = p;
        TurretConstants.CLOSED_LOOP.kI = i;
        TurretConstants.CLOSED_LOOP.kD = d;
        TurretConstants.CLOSED_LOOP.kV = kV;
        TurretConstants.CLOSED_LOOP.kS = kS;
        
        turretMotor.getConfigurator().apply(TurretConstants.CONFIG.withSlot0(TurretConstants.CLOSED_LOOP));

    }
}
