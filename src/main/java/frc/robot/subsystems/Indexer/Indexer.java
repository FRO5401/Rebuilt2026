// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import frc.robot.subsystems.Indexer.IndexerIO.IndexerIOInputs;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private IndexerIOInputs inputs = new IndexerIOInputs();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setIndexer(double spindexerVoltage, double kickerVoltage) {
        io.setSpindexerPercent(spindexerVoltage);
        io.setKickerVoltage(kickerVoltage);
    }

    public Command setIndexerCommand(Supplier<Double> spindexerPercent, Supplier<Double> kickerVoltage) {
        return Commands.runOnce(() -> setIndexer(spindexerPercent.get(), kickerVoltage.get()));
    }

}
