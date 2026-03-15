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

    public AutoRoutine simpleShoot() {
        AutoRoutine routine = autoFactory.newRoutine("Simple Shoot");

        /*split up so apriltag can be read at all times it ends and it can adjust and
        be accurate, since the camera doesn't do well while moving*/
        AutoTrajectory startTraject = routine.trajectory("StartPathRight");
        AutoTrajectory fetchBalls = routine.trajectory("FetchBalls");
        AutoTrajectory fetchBalls2 = routine.trajectory("fetchBalls2");
        AutoTrajectory backtoGoal = routine.trajectory("ReturnToGoal");
        AutoTrajectory aimAtGoal = routine.trajectory("AimAtGoal");

        routine.active().onTrue(
            startTraject.cmd()
        );

        startTraject.done().onTrue(
            Commands.sequence(
                ballSystem.setFlywheel(true),
                ballSystem.setShoot(true),  
                ballSystem.setSpeed(goalTracker.getFlywheelSpeed(true)),
                ballSystem.updateCommand().withTimeout(5),
                ballSystem.resetBallSystem(),
                fetchBalls.cmd()
            )
        );

        fetchBalls.done().onTrue(
            fetchBalls2.cmd()
            //Commands.sequence(
                //Commands.waitSeconds(1),
                //fetchBalls2.cmd()
            //)
        );

        fetchBalls2.done().onTrue(
            backtoGoal.cmd()
            //Commands.sequence(
            //    //Commands.waitSeconds(1),
            //    backtoGoal.cmd()
            //)
        );

        backtoGoal.done().onTrue(
            aimAtGoal.cmd()
            //Commands.sequence(
                //Commands.waitSeconds(1),
            //    aimAtGoal.cmd()
            //)
        );

        aimAtGoal.done().onTrue(
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
