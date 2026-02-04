// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Utilities.ChoreoDrive;

public class Drivetrain extends SubsystemBase {

  SparkMax fL = new SparkMax(0, null);
  SparkMax fR = new SparkMax(0, null);
  SparkMax bL = new SparkMax(0, null);
  SparkMax bR = new SparkMax(0, null);

  ChoreoDrive driveSystem = new ChoreoDrive("Cam", new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));

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

  @Override
  public void periodic() {
    driveSystem.periodic();
    
    // This method will be called once per scheduler run
  }
}
