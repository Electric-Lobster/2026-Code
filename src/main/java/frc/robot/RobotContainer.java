// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {

  Joystick lJoystick = new Joystick(0);
  Joystick rJoystick = new Joystick(1);
  XboxController controller = new XboxController(2);

  AutoChooser m_AutoChooser = new AutoChooser();

  Drivetrain m_Drivetrain = new Drivetrain(lJoystick, rJoystick);

  AutoContainer m_AutoContainer = new AutoContainer(m_Drivetrain);

  public RobotContainer() {
    configureBindings();

    configureChooser();
  }

  private void configureBindings() {}

  private void configureChooser() {
    SmartDashboard.putData(m_AutoChooser);

    m_AutoChooser.addRoutine("Test 1: Straight", m_AutoContainer::testRoutine);
    m_AutoChooser.select("Test 1: Straight");
  }

  public Command getAutonomousCommand() {
    return m_AutoChooser.selectedCommand();
  }
}
