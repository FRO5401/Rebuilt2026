package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Turret.Turret;

public class Autos {
    AutoFactory autoFactory;
    Turret turret;

    public Autos(CommandSwerveDrivetrain drivetrain, Turret turret) {
        this.turret = turret;
        autoFactory = new AutoFactory(
            drivetrain::getPose,
            drivetrain::resetPose,
            drivetrain::followTrajectory,
            true,
            drivetrain
            );
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

        testTraj.atTime("target").onTrue(Commands.runOnce(()->turret.setTarget(new Pose2d(4.5, 4, new Rotation2d()))));

        return routine;
    }
}
