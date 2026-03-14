// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoChooser;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.DefaultBallSystem;
import frc.robot.Commands.DefaultDrive;
import frc.robot.Commands.Test.BallSystemFunctionTest;
import frc.robot.Commands.Test.BallSystemTestBasic;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utilities.GoalTracker;

public class RobotContainer {

  Joystick lJoystick = new Joystick(0);
  Joystick rJoystick = new Joystick(1);
  XboxController controller = new XboxController(2);


  Drivetrain m_Drivetrain = new Drivetrain();

  BallSystem m_BallSystem = new BallSystem();


  AutoChooser m_AutoChooser = new AutoChooser();

  SendableChooser<Command> chooser = new SendableChooser<Command>();

  GoalTracker m_GoalTracker = new GoalTracker(m_Drivetrain);

  AutoContainer m_AutoContainer = new AutoContainer(m_Drivetrain);

  
  public RobotContainer() {

    configureBindings();

    configureChooser();
  }

  private void configureBindings() {
    m_Drivetrain.setDefaultCommand(new DefaultDrive(m_Drivetrain, m_GoalTracker, lJoystick, rJoystick));
    m_BallSystem.setDefaultCommand(new DefaultBallSystem(m_BallSystem, m_GoalTracker, lJoystick, rJoystick, controller));
  }

  private void configureChooser() {
    SmartDashboard.putData(chooser);

    chooser.setDefaultOption("Simple Shoot", m_AutoContainer.simpleShoot());

    chooser.addOption("Simple Shoot", m_AutoContainer.simpleShoot());
    

    //m_AutoChooser.addRoutine("SimpleShoot", m_AutoContainer::simpleShoot);
    //m_AutoChooser.select("SimpleShoot");
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public Command getTestCommand() {
    return Commands.sequence(
      new BallSystemTestBasic(m_BallSystem, 7),
      new BallSystemFunctionTest(m_BallSystem, 5)
    );
  }
}
