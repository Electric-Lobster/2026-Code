// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BallSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BallSystemTestBasic extends Command {

  BallSystem ballSystem;

  Timer timer = new Timer();

  int motorsTested = 0;

  double testTime = 0;

  /** Creates a new BallSystemTestBasic. */
  public BallSystemTestBasic(BallSystem ballSystem, double testTime) {
    this.ballSystem = ballSystem;
    this.testTime = Math.max(testTime, 2);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ballSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (timer.advanceIfElapsed(testTime)) {
      motorsTested += 1;
      ballSystem.runIntake(0);
      ballSystem.runIndexer(0);
      ballSystem.runElevator(0);
      ballSystem.runFlywheel(0);
    }

    double throttle = timer.get() / testTime;

    switch (motorsTested) {
      case 0:
        ballSystem.runIntake(throttle);
        break;
      
      case 1:
        ballSystem.runIndexer(throttle);
        break;
      
      case 2:
        ballSystem.runElevator(throttle);
        break;
      
      case 3:
        ballSystem.runFlywheel(throttle * 14);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSystem.runIntake(0);
    ballSystem.runIndexer(0);
    ballSystem.runElevator(0);
    ballSystem.runFlywheel(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return motorsTested == 4;
  }
}
