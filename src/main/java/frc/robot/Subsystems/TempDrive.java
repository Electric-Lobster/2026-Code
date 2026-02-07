// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TempDrive extends SubsystemBase {

  VictorSPX fL = new VictorSPX(2);
  VictorSPX fR = new VictorSPX(3);
  VictorSPX bL = new VictorSPX(1);
  VictorSPX bR = new VictorSPX(4);

  /** Creates a new TempDrive. */
  public TempDrive() {

    bL.follow(fL);
    bR.follow(fR);

    fR.setInverted(true);
    bR.setInverted(true);

    fL.setNeutralMode(NeutralMode.Coast);
    fR.setNeutralMode(NeutralMode.Coast);
    bL.setNeutralMode(NeutralMode.Coast);
    bR.setNeutralMode(NeutralMode.Coast);

  }

  public void tankdrive(double left, double right) {
    fL.set(ControlMode.PercentOutput, left);
    fR.set(ControlMode.PercentOutput, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
