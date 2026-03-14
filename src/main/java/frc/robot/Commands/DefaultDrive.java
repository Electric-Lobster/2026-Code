// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utilities.GoalTracker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDrive extends Command {
  Drivetrain drivetrain;

  GoalTracker goalTracker;

  Joystick leftJoy;
  Joystick rightJoy;

  /** Creates a new DefaultDrive. */
  public DefaultDrive(Drivetrain drivetrain, GoalTracker goalTracker, Joystick lJoy, Joystick rJoy) {
    this.drivetrain = drivetrain;
    this.goalTracker = goalTracker;
    this.leftJoy = lJoy;
    this.rightJoy = rJoy;
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double trackOffset = 0;
    boolean trackTarget = rightJoy.getRawButton(2);
    boolean intake = leftJoy.getRawButton(1);

    if (trackTarget) {
      trackOffset = drivetrain.getTargetLocateVal(goalTracker.getGoalRelativeAngle());
    }

    double left = leftJoy.getY() * -1;
    double right = rightJoy.getY() * -1;

    if (trackTarget) {
      left = rightJoy.getTwist() * 0.5;
      right = -rightJoy.getTwist() * 0.5;
    }

    left = left * Math.abs(left) * (intake ? 0.5:1);
    right = right * Math.abs(right) * (intake ? 0.5:1);

    SmartDashboard.putNumber("Track Angle", trackOffset);

    drivetrain.tankDrive(left, right);
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
