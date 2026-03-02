package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Autos(CommandSwerveDrivetrain drivetrain, Turret turret, Intake intake, Shooter shooter, Indexer indexer){
        this.turret = turret;
        this.intake = intake;
        this.shooter = shooter;
        this.indexer = indexer;
        autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            true,
            drivetrain
            );
        autoFactory.bind("Intake", Commands.runOnce(()-> intake.setIntake(90, 80), intake));
        autoFactory.bind("IntakeOff", intake.setInfeedVelocityCommand(0));
        autoFactory.bind("Init", shooter.setVelocity(
        () -> RotationsPerSecond
            .of(ShooterConstants.TREE_MAP.get(MathHelp.findDistance(turret.getPoseDifference()).baseUnitMagnitude())),
        intake::getDesiredAngle));
    }

    public AutoRoutine testAuto(){
        AutoRoutine routine = autoFactory.newRoutine("test");

        AutoTrajectory testTraj = routine.trajectory("PickupScore");

        routine.active().onTrue(
            Commands.sequence(
                testTraj.resetOdometry(),
                testTraj.cmd()
            )
        );

        testTraj.atTime("target").onTrue(Commands.runOnce(()->turret.setTarget(new Pose2d(4.5, 4, new Rotation2d()))).withName("target"));


        return routine;
    }

    public AutoRoutine leftBumpAuto(){
        AutoRoutine routine = autoFactory.newRoutine("leftBumpAuto");

        AutoTrajectory testTraj = routine.trajectory("LeftBumpSweep");

        routine.active().onTrue(
            Commands.sequence(
                testTraj.resetOdometry(),
                testTraj.cmd()
            )
        );

        testTraj.atTime("Shoot").onTrue(turret.setSmartTarget().andThen(intake.setPivotPositionCommand(IntakeConstants.INTAKE_OUT_POSE/2)).andThen(indexer.setIndexerCommand(()->7.0, ()-> 11.0)));



        return routine;
    }
}
