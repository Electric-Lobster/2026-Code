// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.Autos.AutoShoot;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utilities.GoalTracker;

/** Add your docs here. */
public class AutoContainer {

    //Subsystems
    Drivetrain drivetrain;
    BallSystem ballSystem;
    GoalTracker goalTracker;

    /**
     * https://choreo.autos/choreolib/auto-factory/
     */
    AutoFactory autoFactory;

    public AutoContainer(Drivetrain m_DriveTrain, BallSystem m_BallSystem, GoalTracker goalTracker) {
        this.drivetrain = m_DriveTrain;
        this.ballSystem = m_BallSystem;
        this.goalTracker = goalTracker;

        autoFactory = new AutoFactory(
            drivetrain.driveSystem::getPose,
            drivetrain.driveSystem::resetPose,
            drivetrain.driveSystem::followTrajectory,
            true,
            drivetrain);
        
        autoFactory.bind("Intake On", ballSystem.setIntake(true));
        autoFactory.bind("Intake Off", ballSystem.setIntake(false));
        autoFactory.bind("Shoot On", ballSystem.setShoot(true));
        autoFactory.bind("Shoot Off", ballSystem.setShoot(false));
        autoFactory.bind("Flywheel On", ballSystem.setFlywheel(true));
        autoFactory.bind("Flywheel Off", ballSystem.setFlywheel(false));
        autoFactory.bind("Reset BallSystem", ballSystem.resetBallSystem());
    }

    /*Depricated? Testing purpouses more likely;
        nevertheless, make sure to add the choreo course before using*/
    public AutoRoutine testRoutine() {
        //Creates Routine
        AutoRoutine routine = autoFactory.newRoutine("Test");

        //Creates Trajectories
        AutoTrajectory testTraject = routine.trajectory("Test 1");

        AutoTrajectory traject2 = routine.trajectory("Test 2");

        //Starts trajectory
        routine.active().onTrue(
            Commands.sequence(
                testTraject.resetOdometry(),
                testTraject.cmd(),
                traject2.cmd()
            )
        );

        return routine;
    }


    public AutoRoutine startLeft() {
        AutoRoutine routine = autoFactory.newRoutine("StartLeft");
        
        AutoTrajectory start = routine.trajectory("StartLeft");
        AutoTrajectory bumpStart = routine.trajectory("BumpLeft");
        AutoTrajectory intake = routine.trajectory("IntakeLeft");
        AutoTrajectory bumpEnd = routine.trajectory("ReturnBumpRight");
        AutoTrajectory aim = routine.trajectory("EndRight");

        routine.active().onTrue(
            Commands.sequence(
                new AutoShoot(drivetrain, ballSystem, goalTracker),
                Commands.parallel(
                    start.cmd(),
                    ballSystem.updateCommand()
                )
            )
        ) ;

        start.done().onTrue(
            bumpStart.cmd()
        );

        bumpStart.done().onTrue(
            intake.cmd()
        );

        intake.done().onTrue(
            bumpEnd.cmd()
        );

        bumpEnd.done().onTrue(
            Commands.parallel(
                aim.cmd(),
                ballSystem.targetSpeed(goalTracker)
            )
        );

        return routine;

    }

    public AutoRoutine startMiddleLeft() {
        AutoRoutine routine = autoFactory.newRoutine("StartMiddleLeft");
        
        AutoTrajectory start = routine.trajectory("StartMiddleLeft");
        AutoTrajectory bumpStart = routine.trajectory("BumpLeft");
        AutoTrajectory intake = routine.trajectory("IntakeLeft");
        AutoTrajectory bumpEnd = routine.trajectory("ReturnBumpRight");
        AutoTrajectory aim = routine.trajectory("EndRight");

        routine.active().onTrue(
            Commands.sequence(
                new AutoShoot(drivetrain, ballSystem, goalTracker),
                Commands.parallel(
                    start.cmd(),
                    ballSystem.updateCommand()
                )
            )
        );

        start.done().onTrue(
            bumpStart.cmd()
        );

        bumpStart.done().onTrue(
            intake.cmd()
        );

        intake.done().onTrue(
            bumpEnd.cmd()
        );

        bumpEnd.done().onTrue(
            Commands.parallel(
                aim.cmd(),
                ballSystem.targetSpeed(goalTracker)
            )
        );

        return routine;
    }

    public AutoRoutine startMiddleRight() {
        AutoRoutine routine = autoFactory.newRoutine("StartMiddleRight");
        
        AutoTrajectory start = routine.trajectory("StartMiddleRIght");
        AutoTrajectory bumpStart = routine.trajectory("BumpRight");
        AutoTrajectory intake = routine.trajectory("IntakeRight");
        AutoTrajectory bumpEnd = routine.trajectory("ReturnBumpLeft");
        AutoTrajectory aim = routine.trajectory("EndLeft");

        routine.active().onTrue(
            Commands.sequence(
                new AutoShoot(drivetrain, ballSystem, goalTracker),
                Commands.parallel(
                    start.cmd(),
                    ballSystem.updateCommand()
                )
            )
        );

        start.done().onTrue(
            bumpStart.cmd()
        );

        bumpStart.done().onTrue(
            intake.cmd()
        );

        intake.done().onTrue(
            bumpEnd.cmd()
        );

        bumpEnd.done().onTrue(
            Commands.parallel(
                aim.cmd(),
                ballSystem.targetSpeed(goalTracker)
            )
        );

        return routine;
    }

    public AutoRoutine startRight() {
        AutoRoutine routine = autoFactory.newRoutine("StartRight");
        
        AutoTrajectory start = routine.trajectory("StartRight");
        AutoTrajectory bumpStart = routine.trajectory("BumpRight");
        AutoTrajectory intake = routine.trajectory("IntakeRight");
        AutoTrajectory bumpEnd = routine.trajectory("ReturnBumpLeft");
        AutoTrajectory aim = routine.trajectory("EndLeft");

        routine.active().whileTrue(
            ballSystem.updateCommand()
        );

        routine.active().onTrue(
            Commands.sequence(
                new AutoShoot(drivetrain, ballSystem, goalTracker),
                start.cmd()
            )
        );

        start.done().onTrue(
            Commands.parallel(
                bumpStart.cmd(),
                ballSystem.setIntake(true)
            )
        );

        bumpStart.done().onTrue(
            intake.cmd()
        );

        intake.done().onTrue(
            bumpEnd.cmd()
        );

        bumpEnd.done().onTrue(
            Commands.parallel(
                aim.cmd(),
                ballSystem.targetSpeed(goalTracker)
            )
        );

        return routine;
    }
}

