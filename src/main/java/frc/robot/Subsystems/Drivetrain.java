// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Utilities.ChoreoDrive;
import static frc.robot.Constants.DriveTrainConstants.*;

public class Drivetrain extends SubsystemBase {

  SparkMax fL = new SparkMax(FLDrive, MotorType.kBrushless);
  SparkMax fR = new SparkMax(FRDrive, MotorType.kBrushless);
  SparkMax bL = new SparkMax(BLDrive, MotorType.kBrushless);
  SparkMax bR = new SparkMax(BRDrive, MotorType.kBrushless);

  public ChoreoDrive driveSystem = new ChoreoDrive("FrameCam", new Transform3d(0.3048, -0.3302, 0.6604, new Rotation3d(0, 0, 0)));

  /** Creates a new Drivetrain. */
  public Drivetrain(Joystick lJoystick, Joystick rJoystick) {

    driveSystem.ConfigureMotors(fL, fR, bL, bR);

    setDefaultCommand(new DefaultDrive(this, lJoystick, rJoystick));
  }

  public void tankDrive(double left, double right) {
    driveSystem.tankDrive(left, right);
  }

  public void trajectoryFollow(DifferentialSample sample) {
    driveSystem.followTrajectory(sample);
  }

  public Pose2d getRobotPose() {
    return driveSystem.getPose();
  }

  @Override
  public void periodic() {
    driveSystem.periodic();

    Transform2d vectToGoal = ShooterConstants.REDGOAL.minus(getRobotPose());

    double distx = vectToGoal.getX();
    double disty = vectToGoal.getY();

    double distance = Math.sqrt( (distx * distx) + (disty * disty));
    //pythagoris hates me


    SmartDashboard.putNumber("Data", distance);
    
    // This method will be called once per scheduler run
  }
}
