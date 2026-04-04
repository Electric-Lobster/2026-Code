// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Utilities.GoalTracker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {

  BallSystem ballSystem;
  GoalTracker goalTracker;

  Timer timer = new Timer();
  int state = 0;

  /** Creates a new Shoot. */
  public Shoot(BallSystem ballSystem, GoalTracker goalTracker) {
    this.ballSystem = ballSystem;
    this.goalTracker = goalTracker;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (timer.advanceIfElapsed(1)) {
      state += 1;
    }

    switch (state) {
      case 0:
          ballSystem.flywheelOn = true;
          ballSystem.flywheelDesiredSpeed = goalTracker.getFlywheelSpeed(false);
        break;

      case 1:
          ballSystem.shoot = true;
          ballSystem.flywheelDesiredSpeed = goalTracker.getFlywheelSpeed(false);
        break;
    
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSystem.shoot = false;
    ballSystem.flywheelOn = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state == 2;
  }
}
