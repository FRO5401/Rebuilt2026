package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    CommandSwerveDrivetrain drivetrain;

    public Autos(CommandSwerveDrivetrain drivetrain, Turret turret, Intake intake,
            Shooter shooter, Indexer indexer) {
        this.turret = turret;
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        autoFactory = new AutoFactory(drivetrain::getPose, drivetrain::resetPose,
                drivetrain::followTrajectory, true, drivetrain);
        autoFactory.bind("Intake", (intake
                .setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)
                .andThen(intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED + .3))));

        autoFactory.bind("IntakeOff", intake.setInfeedVelocityCommand(0));

        autoFactory.bind("Init", Commands.repeatingSequence(shooter.setVelocity(
                () -> RotationsPerSecond
                        .of(ShooterConstants.FLYWHEEL_MAP
                                .get(MathHelp.findDistance(turret.getPoseDifference()).baseUnitMagnitude())),
                intake::getDesiredAngle)));

        autoFactory.bind("start", Commands.repeatingSequence(shooter.setVelocity(
                () -> RotationsPerSecond
                        .of(ShooterConstants.FLYWHEEL_MAP
                                .get(MathHelp.findDistance(turret.getPoseDifference()).baseUnitMagnitude())),
                intake::getDesiredAngle)));
    }

    public AutoRoutine testAuto() {
        AutoRoutine routine = autoFactory.newRoutine("test");

        AutoTrajectory testTraj = routine.trajectory("PickupScore");

        routine.active().onTrue(
                Commands.sequence(testTraj.resetOdometry(), testTraj.cmd()));

        testTraj.atTime("target").onTrue(Commands.runOnce(
                () -> turret.setTarget(new Pose2d(4.5, 4, new Rotation2d())))
                .withName("target"));

        return routine;
    }

    public AutoRoutine leftBumpAuto() {
        AutoRoutine routine = autoFactory.newRoutine("leftBumpAuto");

        AutoTrajectory traj = routine.trajectory("LeftBumpSweep");

        routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

        traj.atTime("Shoot").onTrue(turret.setSmartTarget()
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE / 3))
                .andThen(indexer.setIndexerCommand(() -> .8, () -> 11.0)));

        return routine;
    }

    public AutoRoutine leftDoubleTrenchAuto() {
        AutoRoutine routine = autoFactory.newRoutine("leftBumpAuto");

        AutoTrajectory firstGrab = routine.trajectory("LeftTrenchSweep");
        AutoTrajectory secondGrab = routine.trajectory("LeftTrenchSweep2");

        firstGrab.active().onTrue(intake
                .setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)
                .andThen(intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED + .1)));

        routine.active().onTrue(Commands.sequence(firstGrab.resetOdometry(),
                firstGrab.cmd()));

        firstGrab.atTime("Shoot 1").onTrue(new ParallelCommandGroup(turret.setSmartTarget(),
                (indexer.setIndexerCommand(() -> .8, () -> 11.0))));

        firstGrab.done().onTrue(Commands.waitSeconds(3)
                .andThen(indexer.setIndexerCommand(() -> 0.0, () -> 0.0))
                .andThen(secondGrab.cmd()));

        secondGrab.atTime("Shoot 1").onTrue(new ParallelCommandGroup(
                turret.setSmartTarget(),
                Commands.waitSeconds(.1).andThen(
                        indexer.setIndexerCommand(() -> .8, () -> 11.0)))
                .andThen(Commands.waitSeconds(1))
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE
                        * .2)));

        return routine;
    }

    public AutoRoutine leftSingleTrenchAuto() {
        AutoRoutine routine = autoFactory.newRoutine("leftBumpAuto");

        AutoTrajectory firstGrab = routine.trajectory("LeftTrenchSweep");

        firstGrab.active().onTrue(intake
                .setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)
                .andThen(intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED + .1)));

        routine.active().onTrue(
                Commands.sequence(firstGrab.resetOdometry(), firstGrab.cmd()));

        firstGrab.atTime("Shoot 1").onTrue(new ParallelCommandGroup(turret.setSmartTarget(),
                Commands.waitSeconds(.4).andThen(indexer.setIndexerCommand(() -> .8,
                        () -> 11.0))).andThen(
                intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED))
                .andThen(Commands.waitSeconds(4))
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE
                        * .4)));

        return routine;
    }

    public AutoRoutine rightSingleTrenchAuto() {
        AutoRoutine routine = autoFactory.newRoutine("RightTrenchSweep");

        AutoTrajectory firstGrab = routine.trajectory("RightTrenchSweep");

        firstGrab.active().onTrue(intake
                .setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)
                .andThen(intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED + .1)));

        routine.active().onTrue(
                Commands.sequence(firstGrab.resetOdometry(), firstGrab.cmd()));

        firstGrab.atTime("Shoot 1").onTrue(new ParallelCommandGroup(turret.setSmartTarget(),
                Commands.waitSeconds(.4).andThen(indexer.setIndexerCommand(() -> .8,
                        () -> 11.0))).andThen(
                intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE))
                .andThen(Commands.waitSeconds(3))
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE
                        * .3)).andThen(Commands.waitSeconds(2)).andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)));

        return routine;
    }

    public AutoRoutine leftSingleTrenchCloseAuto() {
        AutoRoutine routine = autoFactory.newRoutine("leftBumpAuto");

        AutoTrajectory firstGrab = routine.trajectory("LeftTrenchSweep2");

        firstGrab.active().onTrue(intake
                .setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)
                .andThen(intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED + .1)));

        routine.active().onTrue(
                Commands.sequence(firstGrab.resetOdometry(), firstGrab.cmd()));

        firstGrab.atTime("Shoot 1").onTrue(new ParallelCommandGroup(turret.setSmartTarget(),
                Commands.waitSeconds(.4).andThen(indexer.setIndexerCommand(() -> .8,
                        () -> 11.0))).andThen(
                intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED))
                .andThen(Commands.waitSeconds(4))
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE
                        * .4)));

        return routine;
    }

    public AutoRoutine DepotBumpSwipe() {
        AutoRoutine routine = autoFactory.newRoutine("DepotBumpNoSweep");

        AutoTrajectory traj = routine.trajectory("DepotBumpNoSweep");

        routine.active().onTrue(Commands.sequence(traj.resetOdometry(), traj.cmd()));

        traj.atTime("Shoot").onTrue(turret.setSmartTarget()
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE * .9))
                .andThen(indexer.setIndexerCommand(() -> .8, () -> 11.0)));

        return routine;
    }

    public AutoRoutine depotWithSwipe() {
        AutoRoutine routine = autoFactory.newRoutine("Depot");

        AutoTrajectory traj = routine.trajectory("Depot");
        AutoTrajectory swipe = routine.trajectory("LeftTrenchSweep2");

        traj.active().onTrue(Commands.repeatingSequence(shooter.setVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.FLYWHEEL_MAP.get(
                        MathHelp.findDistance(turret.getPoseDifference())
                                .baseUnitMagnitude())),
                intake::getDesiredAngle))
                .alongWith(intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED + .1).andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE *.9))));

        swipe.active().onTrue(intake
                .setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)
                .andThen(intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED + .1)));

        routine.active().onTrue(
                Commands.sequence(traj.resetOdometry(), traj.cmd()));

        traj.atTime("Shoot 1").onTrue(new ParallelCommandGroup(turret.setSmartTarget().andThen(indexer.setIndexerCommand(() -> -.5,() -> -11.0)),
                Commands.waitSeconds(.4).andThen(indexer.setIndexerCommand(() -> .8,
                        () -> 11.0))).andThen(
                intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED))
                .andThen(Commands.waitSeconds(4))
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE
                        * .4)));

        traj.done().onTrue(swipe.cmd());

        swipe.active().onTrue(indexer.setIndexerCommand(()->0.0, ()->0.0).alongWith(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE)));

        swipe.atTime("Shoot 1").onTrue(new ParallelCommandGroup(turret.setSmartTarget().andThen(indexer.setIndexerCommand(() -> -.5,() -> -11.0)),
                Commands.waitSeconds(.2

                ).andThen(indexer.setIndexerCommand(() -> .8,
                        () -> 11.0))).andThen(
                intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED))
                .andThen(Commands.waitSeconds(2))
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE
                        * .4)));

        return routine;

    }

    public AutoRoutine depotWithoutSwipe() {
        AutoRoutine routine = autoFactory.newRoutine("Depot");

        AutoTrajectory traj = routine.trajectory("Depot");

        traj.active().onTrue(Commands.repeatingSequence(shooter.setVelocity(
                () -> RotationsPerSecond.of(ShooterConstants.FLYWHEEL_MAP.get(
                        MathHelp.findDistance(turret.getPoseDifference())
                                .baseUnitMagnitude())),
                intake::getDesiredAngle))
                .alongWith(intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED + .1).andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE *.9))));

        routine.active().onTrue(
                Commands.sequence(traj.resetOdometry(), traj.cmd()));

        traj.atTime("Stop").onTrue(drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));

        traj.atTime("Shoot 1").onTrue(new ParallelCommandGroup(turret.setSmartTarget(),
                Commands.waitSeconds(.4).andThen(indexer.setIndexerCommand(() -> .8,
                        () -> 11.0))).andThen(
                intake.setInfeedVelocityCommand(
                        IntakeConstants.INTAKE_SPEED))
                .andThen(Commands.waitSeconds(4))
                .andThen(intake.setPivotPositionCommand(
                        IntakeConstants.INTAKE_OUT_POSE
                        * .4)));

        return routine;

    }

}
