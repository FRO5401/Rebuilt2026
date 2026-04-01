// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.robot.Constants.ShooterConstants;

public class ShooterIOTalon implements ShooterIO {
    private final TalonFX shooterMotor = new TalonFX(ShooterConstants.MOTOR_ID);

    private final TalonFX shooterFollowerMotor = new TalonFX(ShooterConstants.SHOOTER2_MOTOR_ID);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public ShooterIOTalon() {
        shooterMotor.getConfigurator().apply(ShooterConstants.CONFIG);

        shooterFollowerMotor.getConfigurator().apply(ShooterConstants.CONFIG);

        shooterFollowerMotor.setControl(new Follower(ShooterConstants.MOTOR_ID, MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.voltage = shooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.current = shooterMotor.getSupplyCurrent().getValueAsDouble();
        inputs.velocity = shooterMotor.getVelocity().getValueAsDouble();
    };

    @Override
    public void applyVoltage(double voltage) {
        shooterMotor.setControl(voltageRequest.withOutput(voltage));
    };

    @Override
    public void setVelocity(double velocity) {
        shooterMotor.setControl(velocityRequest.withVelocity(velocity).withEnableFOC(true));
    };

    @Override
    public void stop() {};

    @Override
    public void applyPID(double P, double I, double D, double S, double V) {
        ShooterConstants.CLOSED_LOOP.kP = P;
        ShooterConstants.CLOSED_LOOP.kI = I;
        ShooterConstants.CLOSED_LOOP.kD = D;
        ShooterConstants.CLOSED_LOOP.kS = S;
        ShooterConstants.CLOSED_LOOP.kV = V;

        shooterMotor.getConfigurator().apply(ShooterConstants.CONFIG);
    }

}
