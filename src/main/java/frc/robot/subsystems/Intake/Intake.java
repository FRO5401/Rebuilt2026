// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeIO.InfeedIOInputs;
import frc.robot.subsystems.Intake.IntakeIO.PivotIOInputs;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private final InfeedIOInputsAutoLogged infeedInputs = new InfeedIOInputsAutoLogged();

    /** Creates a new Intake. */
    public Intake(IntakeIO m_io) {
        super("Intake");
        this.io = m_io;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        io.updateIntakeInputs(pivotInputs, infeedInputs);
        Logger.processInputs("Intake", pivotInputs);
        Logger.processInputs("Intake", infeedInputs);
  }
}
