// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterIOTalon implements ShooterIO {
    TalonFX shooterMotor = new TalonFX(ShooterConstants.MOTOR_ID);
    
    VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    VoltageOut voltageRequest = new VoltageOut(0.0);

    
    
    public ShooterIOTalon(){
        shooterMotor.getConfigurator().apply(ShooterConstants.CONFIG);

        
    }
    public void updateInputs(ShooterIOInputs inputs){
        inputs.voltage = shooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.current = shooterMotor.getSupplyCurrent().getValueAsDouble();
        inputs.velocity = shooterMotor.getVelocity().getValueAsDouble();
    };

    public void applyVoltage(double voltage){
        shooterMotor.setControl(voltageRequest.withOutput(voltage));
    };

    public void setVelocity(double velocity, ShooterIOInputs inputs){
        shooterMotor.setControl(velocityRequest.withVelocity(velocity));
    };
    
    public void stop(){};

    public void applyPID(double P, double I, double D, double S, double V){
        ShooterConstants.CLOSED_LOOP.kP = P;
        ShooterConstants.CLOSED_LOOP.kI = I;
        ShooterConstants.CLOSED_LOOP.kD = D;
        ShooterConstants.CLOSED_LOOP.kS = S;
        ShooterConstants.CLOSED_LOOP.kV = V;
        
        shooterMotor.getConfigurator().apply(ShooterConstants.CONFIG);
    }

}
