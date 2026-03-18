// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.ChoreoDrive;
import static frc.robot.Constants.DriveTrainConstants.*;

public class Drivetrain extends SubsystemBase {

  SparkMax fL = new SparkMax(FLDrive, MotorType.kBrushless);
  SparkMax fR = new SparkMax(FRDrive, MotorType.kBrushless);
  SparkMax bL = new SparkMax(BLDrive, MotorType.kBrushless);
  SparkMax bR = new SparkMax(BRDrive, MotorType.kBrushless);

  PIDController anglePID = new PIDController(0, 0, 0);
  SimpleMotorFeedforward feedFoward = new SimpleMotorFeedforward(0, 0);

  public ChoreoDrive driveSystem = new ChoreoDrive("FrameCam", new Transform3d(0.3048, -0.3302, 0.6604, new Rotation3d(0, 20, 0)));

  /** Creates a new Drivetrain. */
  public Drivetrain() {

    driveSystem.ConfigureMotors(fL, fR, bL, bR);

    anglePID.setTolerance(3);
  }

  public void tankDrive(double left, double right) {
    driveSystem.tankDrive(left, right);
  }

  public double getTargetLocateVal(double angle) {
    double pidVal = anglePID.calculate(getRobotPose().getRotation().getDegrees(), angle);
    double feedFowardVal = feedFoward.calculateWithVelocities(getRobotPose().getRotation().getDegrees(), angle);

    double finalVal = feedFowardVal + pidVal;

    SmartDashboard.putBoolean("CorrectAim", anglePID.atSetpoint());

    return finalVal;
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

    if (!SmartDashboard.containsKey("P")) { SmartDashboard.putNumber("P", 0); }
    if (!SmartDashboard.containsKey("I")) { SmartDashboard.putNumber("I", 0); }
    if (!SmartDashboard.containsKey("D")) { SmartDashboard.putNumber("D", 0); }
    if (!SmartDashboard.containsKey("S")) { SmartDashboard.putNumber("S", 0); }
    if (!SmartDashboard.containsKey("V")) { SmartDashboard.putNumber("V", 0); }

    anglePID.setP(SmartDashboard.getNumber("P", 0));
    anglePID.setI(SmartDashboard.getNumber("I", 0));
    anglePID.setD(SmartDashboard.getNumber("D", 0));
    feedFoward.setKs(SmartDashboard.getNumber("S", 0));
    feedFoward.setKv(SmartDashboard.getNumber("V", 0));
    
    // This method will be called once per scheduler run
  }
}
