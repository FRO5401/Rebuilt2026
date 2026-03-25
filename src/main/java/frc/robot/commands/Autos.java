package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utils.MathHelp;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer.Indexer;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Turret.Turret;

public class Autos {
    AutoFactory autoFactory;
    Turret turret;
    Intake intake;
    Shooter shooter;
    Indexer indexer;

    public Autos(CommandSwerveDrivetrain drivetrain, Turret turret, Intake intake, Shooter shooter, Indexer indexer) {
        this.turret = turret;
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        autoFactory = new AutoFactory(
                drivetrain::getPose,
                drivetrain::resetPose,
                drivetrain::followTrajectory,
                true,
                drivetrain);
        autoFactory.bind("Intake", (intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE).andThen(intake.setInfeedVelocityCommand(IntakeConstants.INTAKE_SPEED-.1))));
        autoFactory.bind("IntakeOff", intake.setInfeedVelocityCommand(0));
        autoFactory.bind("Init", Commands.repeatingSequence(shooter.setVelocity(
                () -> RotationsPerSecond
                        .of(ShooterConstants.FLYWHEEL_MAP
                                .get(MathHelp.findDistance(turret.getPoseDifference()).baseUnitMagnitude())),
                intake::getDesiredAngle)));
    }

    public AutoRoutine testAuto() {
        AutoRoutine routine = autoFactory.newRoutine("test");

        AutoTrajectory testTraj = routine.trajectory("PickupScore");

        routine.active().onTrue(
                Commands.sequence(
                        testTraj.resetOdometry(),
                        testTraj.cmd()));

        testTraj.atTime("target").onTrue(
                Commands.runOnce(() -> turret.setTarget(new Pose2d(4.5, 4, new Rotation2d()))).withName("target"));

        return routine;
    }

    public AutoRoutine leftBumpAuto() {
        AutoRoutine routine = autoFactory.newRoutine("leftBumpAuto");

        AutoTrajectory traj = routine.trajectory("LeftBumpSweep");

        routine.active().onTrue(
                Commands.sequence(
                        traj.resetOdometry(),
                        traj.cmd()));

        traj.atTime("Shoot")
                .onTrue(turret.setSmartTarget()
                        .andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE/3))
                        .andThen(indexer.setIndexerCommand(() -> .8, () -> 11.0)));

        return routine;
    }

    public AutoRoutine leftDoubleTrenchAuto() {
        AutoRoutine routine = autoFactory.newRoutine("leftBumpAuto");

        AutoTrajectory firstGrab = routine.trajectory("LeftTrenchSweep");
        AutoTrajectory secondGrab = routine.trajectory("LeftTrenchSweep2");

        firstGrab.active().onTrue(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE).andThen(intake.setInfeedVelocityCommand(IntakeConstants.INTAKE_SPEED-.1)));


        routine.active().onTrue(
                Commands.sequence(
                        firstGrab.resetOdometry(),
                        
                        firstGrab.cmd()));

        

        firstGrab.atTime("Shoot 1")
                .onTrue(turret.setSmartTarget().andThen(indexer.setIndexerCommand(() -> .8, () -> 11.0))
                        .andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE )).andThen(Commands.waitSeconds(3)));

        firstGrab.done().onTrue(Commands.waitSeconds(4).andThen(indexer.setIndexerCommand(() -> 0.0, () -> 0.0)).andThen(secondGrab.cmd()));

        secondGrab.atTime("Shoot 2")
                .onTrue((indexer.setIndexerCommand(() -> .8, () -> 11.0))
                        .andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE )));

        return routine;
    }

    public AutoRoutine leftSingleTrenchAuto() {
        AutoRoutine routine = autoFactory.newRoutine("leftBumpAuto");

        AutoTrajectory firstGrab = routine.trajectory("LeftTrenchSweep");

        firstGrab.active().onTrue(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE).andThen(intake.setInfeedVelocityCommand(IntakeConstants.INTAKE_SPEED-.1)));

        routine.active().onTrue(

                Commands.sequence(
                        firstGrab.resetOdometry(),
                        firstGrab.cmd()));

        firstGrab.atTime("Shoot 1")
                .onTrue(new ParallelCommandGroup(turret.setSmartTarget(), (indexer.setIndexerCommand(() -> .8, () -> 11.0)))
                        .andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)).andThen(Commands.waitSeconds(2)).andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE*.2)));

        


        return routine;
    }

    public AutoRoutine DepotNoSwipe() {
        AutoRoutine routine = autoFactory.newRoutine("DepotBumpNoSweep");

        AutoTrajectory traj = routine.trajectory("DepotBumpNoSweep");

        routine.active().onTrue(
                Commands.sequence(
                        traj.resetOdometry(),
                        traj.cmd()));

        traj.atTime("Shoot")
                .onTrue(turret.setSmartTarget()
                        .andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE * .9))
                        .andThen(indexer.setIndexerCommand(() -> .8, () -> 11.0)));

        return routine;
    }
    
}
