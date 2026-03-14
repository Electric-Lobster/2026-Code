// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Subsystems.Drivetrain;

/** Add your docs here. */
public class GoalTracker {

    Drivetrain drivetrain;

    public GoalTracker(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /**
     * @return the current alliance, returns red if none is present
     */
    private Alliance getAlliance() {
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get();

        } else {
            return Alliance.Red;

        }
    }

    private Pose2d getGoal() {
        Alliance alliance = getAlliance();

        if (alliance == Alliance.Red) {
            return ShooterConstants.REDGOAL;
        } else {
            return ShooterConstants.BLUEGOAL;
        }
    }

    private Transform2d getVectToGoal() {
        Pose2d robotPose = drivetrain.getRobotPose();
        Pose2d goal = getGoal();

        Transform2d vectToGoal = goal.minus(robotPose);

        return vectToGoal;
    }

    public double getGoalDistance() {
        Transform2d vectToGoal = getVectToGoal();

        double distx = vectToGoal.getX();
        double disty = vectToGoal.getY();

        double distance = Math.sqrt( (distx * distx) + (disty * disty));
        //pythagoris hates me

        SmartDashboard.putNumber("Distance to Goal", distance);

        return distance;
    }

    public double getGoalRelativeAngle() {

        Pose2d robotPose = drivetrain.getRobotPose();
        Pose2d goal = getGoal();

        Transform2d vectToGoal = goal.minus(robotPose);

        return vectToGoal.getTranslation().getAngle().getDegrees() ;
    }

    private double filteredSpeed = 0;
    private double filterRate = 0.9;
    public double getFlywheelSpeed(boolean instant) {
        double speed = ShooterConstants.LOOKUPTABLE.get(getGoalDistance());

        filteredSpeed = (filteredSpeed * filterRate) + (speed * (1 - filterRate));

        if (instant) {
            return speed;
        } else {
            return filteredSpeed;
        }
    }
}
