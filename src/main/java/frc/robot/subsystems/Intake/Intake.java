// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utils.TunableNumber;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final InfeedIOInputsAutoLogged infeedInputs = new InfeedIOInputsAutoLogged();
    //TODO: Finish tuning these, I ball parked it
    private TunableNumber kp = new TunableNumber("Intake/kp", 40);
    private TunableNumber ki = new TunableNumber("Intake/ki", 0, true);
    private TunableNumber kd = new TunableNumber("Intake/kd", 0.936);

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
        Logger.recordOutput("Intake/Intake Pose", new Pose3d(0.215, 0, 0.178, new Rotation3d(0, pivotInputs.angle - Degrees.of(70).in(Radians), 0)));

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
}
