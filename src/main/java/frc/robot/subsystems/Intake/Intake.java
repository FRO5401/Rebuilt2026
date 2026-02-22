// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;



import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.TunableNumber;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final InfeedIOInputsAutoLogged infeedInputs = new InfeedIOInputsAutoLogged();
    private final String key = "Intake/";
    
    //TODO: Finish tuning these, I ball parked it
    private TunableNumber kp = new TunableNumber(key.concat("kp"), 40);
    private TunableNumber kd = new TunableNumber("Intake/kd", 0.936);
    private TunableNumber ki = new TunableNumber(key.concat("ki"), 0, true);

    /** Creates a new Intake. */
    public Intake(IntakeIO m_io) {
        this.io = m_io;
        io.setPivotPID(kp.get(), ki.get(), kd.get());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateIntakeInputs(pivotInputs, infeedInputs);
        Logger.processInputs("Intake/Pivot Inputs", pivotInputs);
        Logger.processInputs("Intake/Infeed Inputs", infeedInputs);

        if(kp.hasChanged() || ki.hasChanged() || kd.hasChanged()){
            io.setPivotPID(kp.get(), ki.get(), kd.get());
        }

    }

   public void setPivotPosition(double angle){
        io.setPivotPosition(angle);

   }

   public void setInfeedVelocity(double percent){
        io.setInfeedVelocity(percent);
   }

   public void setIntake(double angle, double percent){
        setPivotPosition(angle);
        setInfeedVelocity(percent);
   }
   
   public boolean isIntakeDeployed(){
    return pivotInputs.angle<10;
   }

   public double getPivotPosition(){
    return pivotInputs.angle;
   }

   public Command setPivotPositionCommand(double angle){
    return runOnce(()->setPivotPosition(angle));
   }

   public Command setInfeedVelocityCommand(double percent){
    return runOnce(()->setInfeedVelocity(percent));
   }
}
