// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drivetrain;

/** Add your docs here. */
public class AutoContainer {

    //Subsystems
    Drivetrain drivetrain;

    /**
     * https://choreo.autos/choreolib/auto-factory/
     */
    AutoFactory autoFactory;

    public AutoContainer(Drivetrain m_DriveTrain) {
        this.drivetrain = m_DriveTrain;

        autoFactory = new AutoFactory(
            drivetrain.driveSystem::getPose,
            drivetrain.driveSystem::resetPose,
            drivetrain.driveSystem::followTrajectory,
            true,
            drivetrain);
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

    public Command simpleShoot() {
        AutoRoutine routine = autoFactory.newRoutine("Simple Shoot");

        AutoTrajectory startTraject = routine.trajectory("StartPathRight");

        routine.active().onTrue(
            Commands.sequence(
                startTraject.cmd()
            )   
        );

        return routine.cmd();
    }

}
