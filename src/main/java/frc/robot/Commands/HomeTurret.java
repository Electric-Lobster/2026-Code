// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.fasterxml.jackson.annotation.JsonFormat.Shape;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HomeTurret extends Command {

  Shooter shooter;

  int homingState = 0;

  /** Creates a new HomeTurret. */
  public HomeTurret(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    switch (homingState) {
      case 0: // initial Home

        shooter.setTurretThrottle(-1);

        if (shooter.getTurretAtSoftStop()) {
          shooter.resetEncoder();
          homingState = 1;
        }
        
        break;
      
      case 1: // depress switch

        shooter.setTurretThrottle( 1);

        if (shooter.getTurretrelativAngle() > 10) {
          homingState = 2;
        }

        break;

      case 2: // Final Home
        shooter.setTurretThrottle(-0.25);

        if (shooter.getTurretAtSoftStop()) {
          shooter.resetEncoder();
          shooter.setTurretThrottle(0);
          homingState = 3;
        }
        break;
      
        case 3: // point foward
        // Foward angle from limitswitch
        if (shooter.setTurretPosition(135)) {
          shooter.resetEncoder();
          shooter.setTurretThrottle(0);
          homingState = 4;
        }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return homingState == 4;
  }
}
