// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final InfeedIOInputsAutoLogged infeedInputs = new InfeedIOInputsAutoLogged();
    private final Alert pivotMotorsConnected;
    private final Alert infeedMotorConnected;

    /** Creates a new Intake. */
    public Intake(IntakeIO m_io) {
        super("Intake");
        this.io = m_io;

        pivotMotorsConnected = new Alert(getName()+" Pivot Motor Disconnected", AlertType.kWarning);
        infeedMotorConnected = new Alert(getName()+" Infeed Motor Disconnected", AlertType.kWarning);

        setDefaultCommand(setInfeedVoltage(0.0));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateIntakeInputs(pivotInputs, infeedInputs);
        Logger.processInputs("Intake", pivotInputs);
        Logger.processInputs("Intake", infeedInputs);

        pivotMotorsConnected.set(!(pivotInputs.isMasterConnected && pivotInputs.isFollowerConnected));
        infeedMotorConnected.set(!infeedInputs.isConnected);
    }
    
    public void updatePivotPID(double kp, double ki, double kd){
        io.setPivotPID(kp, ki, kd);
    }

    /*  Commands */
    public Command setPivotAngle(double angle){
        return runOnce(()-> io.setPivotPosition(angle));
    }

    public Command setPivotIntaking(double angle, double velocity){
        return runOnce(()->io.setIntake(angle, velocity));
    }

    public Command setInfeedVelocity(double velocity){
        return runOnce(()-> io.setInfeedVelocity(velocity));
    }

    public Command setPivotVoltage(double voltage){
        return runOnce(()-> io.setPivotVoltage(voltage));
    }

    public Command setInfeedVoltage(double voltage){
        return runOnce(()->io.setInfeedVoltage(voltage));
    }

    public Command stop(){
        return runOnce(()-> io.stop());
    }
}
