// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.PhotonVisionProccesing;

public class Shooter extends SubsystemBase {

  // MOTORS
  VictorSPX turretMotor = new VictorSPX(100);

  // Utilities
  Pose2d turretPos = new Pose2d();
  PhotonVisionProccesing turretCam = new PhotonVisionProccesing("pass", new Transform3d(0,0,0, new Rotation3d(0, 0, 0)));
  Field2d turretFieldView = new Field2d();

  /** Creates a new Shooter. */
  public Shooter() {
    SmartDashboard.putData(turretFieldView);

    turretMotor.setNeutralMode(NeutralMode.Brake);
    turretMotor.setInverted(false);

  }


  public void setTurretThrottle(double throttle) {
    turretMotor.set(ControlMode.PercentOutput, throttle);
  }


  public void updateTurretPos(Pose2d robotPose) {
    turretPos = turretCam.GetBestPose(robotPose);

    turretFieldView.setRobotPose(turretPos);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
