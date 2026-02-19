// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.Vector;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.DefaultShooter;
import frc.robot.Utilities.PhotonVisionProccesing;
import pabeles.concurrency.ConcurrencyOps.Reset;

public class Shooter extends SubsystemBase {

  // MOTORS
  VictorSPX turretMotor = new VictorSPX(50);

  // ENCODERS
  Encoder turretEncoder = new Encoder(0, 1);

  // LIMIT SWITCHES
  DigitalInput turretSoftStop = new DigitalInput(9);

  // PIDS
  PIDController turretPID = new PIDController(1, 0, 0);
  PIDController turretPID2 = new PIDController(0.1, 0, 0);

  // Utilities
  Pose2d turretPos = new Pose2d();

  PhotonVisionProccesing turretCam = new PhotonVisionProccesing("Arducam_OV9281_USB_Camera", new Transform3d(0,0,0, new Rotation3d(0, 0, 0)));
  
  Field2d turretFieldView = new Field2d();

  /** Creates a nsew Shooter. */
  public Shooter() {
    SmartDashboard.putData(turretFieldView);

    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(false);

    // pulses per rev * gear ratio * degrees per revolution
    turretEncoder.setDistancePerPulse(7 * 27 * 360);

    //setDefaultCommand(new DefaultShooter(this));

  }


  public void setTurretThrottle(double throttle) {
    turretMotor.set(ControlMode.PercentOutput, throttle);
  }


  public boolean setTurretPosition(double pos) {
    double throttle = turretPID2.calculate(turretEncoder.getDistance(), pos);

    setTurretThrottle(throttle);

    if (turretPID2.atSetpoint()) {
      setTurretThrottle(0);
      return false;
    } else {
      return true;
    }

  }
  

  public void lookAtPosition(Pose2d target) {

    Transform2d posError = target.minus(turretPos);

    double desiredAngle = Math.atan(posError.getY() / posError.getX());

    double throttle = turretPID.calculate(turretPos.getRotation().getRadians(), desiredAngle);

    throttle = MathUtil.clamp(throttle, -0.25, 0.25);

    setTurretThrottle(throttle);

    SmartDashboard.putNumber("throttle", throttle);
  }


  public void updateTurretPos(Pose2d robotPose) {
    turretPos = turretCam.GetBestPose(robotPose);

    turretFieldView.setRobotPose(turretPos);
  }


  public boolean getTurretAtSoftStop() {
    return turretSoftStop.get();
  }

  public double getTurretrelativAngle() {
    return turretEncoder.getDistance();
  }

  public void resetEncoder() {
    turretEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
