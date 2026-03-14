// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public AutoRoutine simpleShoot() {
        AutoRoutine routine = autoFactory.newRoutine("Simple Shoot");

        AutoTrajectory startTraject = routine.trajectory("StartPathRight");
        AutoTrajectory fetchBalls = routine.trajectory("FetchBalls");
        AutoTrajectory fetchBalls2 = routine.trajectory("fetchBalls2");
        AutoTrajectory backtoGoal = routine.trajectory("ReturnToGoal");
        AutoTrajectory aimAtGoal = routine.trajectory("AimAtGoal");

        routine.active().onTrue(
            startTraject.cmd()
        );

        startTraject.active().onFalse(
            Commands.sequence(
                ballSystem.setFlywheel(true),
                ballSystem.setShoot(true),
                ballSystem.setSpeed(goalTracker.getFlywheelSpeed(true)),
                ballSystem.updateCommand().withTimeout(5),
                ballSystem.resetBallSystem(),
                fetchBalls.cmd()
            )
        );

        fetchBalls.active().onFalse(
            Commands.sequence(
                Commands.waitSeconds(1),
                fetchBalls2.cmd()
            )
        );

        fetchBalls2.active().onFalse(
            Commands.sequence(
                Commands.waitSeconds(1),
                backtoGoal.cmd()
            )
        );

        backtoGoal.active().onFalse(
            Commands.sequence(
                Commands.waitSeconds(1),
                aimAtGoal.cmd()
            )
        );

        aimAtGoal.active().onFalse(
            Commands.sequence(
                ballSystem.setFlywheel(true),
                ballSystem.setShoot(true),
                ballSystem.setSpeed(goalTracker.getFlywheelSpeed(true)),
                ballSystem.updateCommand().withTimeout(4),
                ballSystem.resetBallSystem()
            )
        );

        return routine;
    }

}
