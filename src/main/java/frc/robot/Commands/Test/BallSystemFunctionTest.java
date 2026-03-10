// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Test;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.BallSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BallSystemFunctionTest extends Command {

  BallSystem ballSystem;

  Timer timer = new Timer();

  double testTime;

  int functionsTested = 0;

  /** Creates a new BallSystemFunctionTest. */
  public BallSystemFunctionTest(BallSystem ballSystem, double testTime) {
    this.ballSystem = ballSystem;
    this.testTime = Math.max(testTime, 3);

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
      functionsTested += 1;
    }

    switch (functionsTested) {
      case 0:
        SmartDashboard.putString("Function Test", "Function: System off");
        ballSystem.manualIntake = false;
        ballSystem.shoot = false;
        ballSystem.flywheelOn = false;
        ballSystem.flywheelDesiredSpeed = 0;
        break;
      
        case 1:
        SmartDashboard.putString("Function Test", "Function: Manual Intake");
        ballSystem.manualIntake = true;
        ballSystem.shoot = false;
        ballSystem.flywheelOn = false;
        ballSystem.flywheelDesiredSpeed = 0;
        break;

        case 2:
        SmartDashboard.putString("Function Test", "Function: Shoot Only");
        ballSystem.manualIntake = false;
        ballSystem.shoot = true;
        ballSystem.flywheelOn = false;
        ballSystem.flywheelDesiredSpeed = 0;
        break;

        case 3:
        SmartDashboard.putString("Function Test", "Function: Flywheel 0");
        ballSystem.manualIntake = false;
        ballSystem.shoot = false;
        ballSystem.flywheelOn = true;
        ballSystem.flywheelDesiredSpeed = 0;
        break;

        case 4:
        SmartDashboard.putString("Function Test", "Function: Flywheel 10");
        ballSystem.manualIntake = false;
        ballSystem.shoot = false;
        ballSystem.flywheelOn = true;
        ballSystem.flywheelDesiredSpeed = 10;
        break;

        case 5:
        SmartDashboard.putString("Function Test", "Function: Flywheel 50 + shoot");
        ballSystem.manualIntake = false;
        ballSystem.shoot = true;
        ballSystem.flywheelOn = true;
        ballSystem.flywheelDesiredSpeed = 50;
        break;

        case 6:
        SmartDashboard.putString("Function Test", "Function: Flywheel 75 + shoot");
        ballSystem.manualIntake = false;
        ballSystem.shoot = true;
        ballSystem.flywheelOn = true;
        ballSystem.flywheelDesiredSpeed = 75;
        break;

        case 7:
        SmartDashboard.putString("Function Test", "Function: Flywheel 100 + shoot");
        ballSystem.manualIntake = false;
        ballSystem.shoot = true;
        ballSystem.flywheelOn = true;
        ballSystem.flywheelDesiredSpeed = 100;
        break;

        case 8:
        SmartDashboard.putString("Function Test", "Function: Flywheel 0 + intake + shoot");
        ballSystem.manualIntake = true;
        ballSystem.shoot = true;
        ballSystem.flywheelOn = true;
        ballSystem.flywheelDesiredSpeed = 0;
        break;

        case 9:
        SmartDashboard.putString("Function Test", "Function: Flywheel 100 + shoot");
        ballSystem.manualIntake = false;
        ballSystem.shoot = true;
        ballSystem.flywheelOn = true;
        ballSystem.flywheelDesiredSpeed = 100;
        break;

        case 10:
        SmartDashboard.putString("Function Test", "Function: System Off");
        ballSystem.manualIntake = false;
        ballSystem.shoot = false;
        ballSystem.flywheelOn = false;
        ballSystem.flywheelDesiredSpeed = 0;
        break;
    }

    SmartDashboard.putNumber("Functions Tested", functionsTested);
    ballSystem.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("Function Test", "Not Testing");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return functionsTested > 10;
  }
}
