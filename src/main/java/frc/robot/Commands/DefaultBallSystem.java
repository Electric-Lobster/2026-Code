// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Utilities.GoalTracker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultBallSystem extends Command {

  BallSystem ballSystem;

  GoalTracker goalTracker;

  Joystick leftJoy;
  Joystick rightJoy;
  XboxController controller;

  /** Creates a new DefaultBallSystem. */
  public DefaultBallSystem(BallSystem ballSystem, GoalTracker goalTracker, Joystick leftjoy, Joystick rightjoy, XboxController controller) {
    this.ballSystem = ballSystem;
    this.goalTracker = goalTracker;
    this.leftJoy = leftjoy;
    this.rightJoy = rightjoy;
    this.controller = controller;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    //TODO remove this
    if ( !SmartDashboard.containsKey("Shoot") ) { SmartDashboard.putBoolean("Shoot", false); }
    if ( !SmartDashboard.containsKey("Reverse") ) { SmartDashboard.putBoolean("Reverse", false); }
    if ( !SmartDashboard.containsKey("Flywheel") ) { SmartDashboard.putBoolean("Flywheel", false); }
    if ( !SmartDashboard.containsKey("Intake") ) { SmartDashboard.putBoolean("Intake", false); }
    if ( !SmartDashboard.containsKey("Speed") ) { SmartDashboard.putNumber("Speed", 0); }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean intake = leftJoy.getRawButton(1);
    boolean reverse = leftJoy.getRawButton(2);

    boolean flywheel = rightJoy.getRawButtonPressed(3);
    boolean trackGoal = rightJoy.getRawButton(2);
    boolean shoot = rightJoy.getRawButton(1);
    double manualShooterSpeed = ((-rightJoy.getRawAxis(3) + 1) / 2.0) * 100;


    ballSystem.manualIntake = intake;
    ballSystem.shoot = shoot;
    ballSystem.reverseIndexer = reverse;
    if (flywheel) {
      ballSystem.flywheelOn = !ballSystem.flywheelOn;
    }
    
    if (trackGoal) {
      ballSystem.flywheelDesiredSpeed = goalTracker.getFlywheelSpeed();
    } else {
      ballSystem.flywheelDesiredSpeed = manualShooterSpeed;
    }

    ballSystem.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
