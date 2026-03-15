// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import com.fasterxml.jackson.databind.deser.DataFormatReaders.Match;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BallSystem;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Utilities.GoalTracker;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Driveback extends Command {

  Drivetrain drivetrain;
  BallSystem ballSystem;

  GoalTracker goalTracker;

  Timer timer = new Timer();

  int state = 0;

  /** Creates a new Driveback. */
  public Driveback(Drivetrain drivetrain, BallSystem ballSystem, GoalTracker goalTracker) {
    this.drivetrain = drivetrain;
    this.ballSystem = ballSystem;
    this.goalTracker = goalTracker;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(1.5)) { state ++; }

    switch (state) {
      case 0:
        drivetrain.tankDrive(-0.25, -0.25);
        break;
    
      case 1:
        drivetrain.tankDrive(0, 0);
        break;

      case 2:
        ballSystem.flywheelOn = true;
        ballSystem.shoot = true;
        ballSystem.flywheelDesiredSpeed = goalTracker.getFlywheelSpeed(true);
        break;

      case 3:
        ballSystem.flywheelOn = false;
        ballSystem.shoot = false;

        drivetrain.tankDrive(0.25, -0.25);
        break;

      case 4:
        drivetrain.tankDrive(0.5, 0.5);
        break;

      case 5:
        drivetrain.tankDrive(0.5, 0.5);
        ballSystem.manualIntake = true;
        break;
      
      case 6:
        drivetrain.tankDrive(-0.5, 0.5);
        ballSystem.manualIntake = false;
        break;
      
      case 7:
        drivetrain.tankDrive(-0.5, -0.5);
        break;
      
      case 8:
        drivetrain.tankDrive(0, 0);
        break;

      default:
        break;
    }

    SmartDashboard.putNumber("Auto State", state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return state > 8;
  }
}
