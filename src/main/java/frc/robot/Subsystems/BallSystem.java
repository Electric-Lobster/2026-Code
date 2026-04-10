// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.BallSystemConstants.*;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Utilities.GoalTracker;

public class BallSystem extends SubsystemBase {

  //MOTORS
  SparkMax intake = new SparkMax(INTAKE, MotorType.kBrushless);
  SparkMax indexer = new SparkMax(INDEXER, MotorType.kBrushless);
  SparkMax elevator = new SparkMax(ELEVATOR, MotorType.kBrushed);
  TalonFX flywheel = new TalonFX(FLYWHEEL);

  //Flywheel Control
  PIDController flywheelPID = new PIDController(1, 1, 0.15);
  SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(1.5, 0.1);

  //System control variables
  public boolean manualIntake = false;

  public boolean shoot = false;

  public boolean reverseIndexer = false;

  public boolean flywheelOn = false;
  public double flywheelDesiredSpeed = 0;

  private double filteredVelocity = 0;
  

  /** Creates a new Shooter. */
  public BallSystem() {

    SparkMaxConfig intakeConfig = new SparkMaxConfig();
    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();

    intakeConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    intakeConfig.smartCurrentLimit(INTAKECURRANTLIMITS[0], INTAKECURRANTLIMITS[1], INTAKECURRANTLIMITS[2]).inverted(INTAKEINVERT).idleMode(IdleMode.kBrake);

    indexerConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    indexerConfig.smartCurrentLimit(INDEXERCURRANTLIMITS[0], INDEXERCURRANTLIMITS[1], INDEXERCURRANTLIMITS[2]).inverted(INDEXERINVERT).idleMode(IdleMode.kBrake);

    elevatorConfig.smartCurrentLimit(ELEVATORCURRANTLIMITS[0], ELEVATORCURRANTLIMITS[1], ELEVATORCURRANTLIMITS[2]).inverted(ELEVATORINVERT).idleMode(IdleMode.kBrake);

    intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    flywheel.getConfigurator().apply( new MotorOutputConfigs().withInverted(FLYWHEELINVERT).withNeutralMode(NeutralModeValue.Brake));

    flywheelPID.setTolerance(4);
  }


  public void runIntake(double throttle) {
    
    intake.set(throttle);

  }

  public void runIndexer(double throttle) {

    indexer.set(throttle);

  }

  public void runElevator(double throttle) {

    elevator.set(throttle);

  }

  public void runFlywheel(double voltage) {

    voltage = MathUtil.clamp(voltage, 0, 14);

    flywheel.setVoltage(voltage);

  }


  public void setFlywheelSpeed(double speed) {

    double pidVal = flywheelPID.calculate(filteredVelocity, speed);
    double feedfowardVal = flywheelFeedforward.calculateWithVelocities(filteredVelocity, speed);

    runFlywheel(pidVal + feedfowardVal);

    SmartDashboard.putNumber("Flywheel Desired Speed", speed);
    SmartDashboard.putNumber("Flywheel Throttle", pidVal + feedfowardVal);
    SmartDashboard.putNumber("Flywheel Error", speed - filteredVelocity);
  }


  public boolean flywheelAtSpeed() {
    return flywheelPID.atSetpoint() && flywheelDesiredSpeed > 1 && flywheelOn;
  }


  public void update() {

    updateIntake();

    updateIndexer();

    updateElevator();

    updateFlywheel();

  }

  private void updateIntake() {
    if (reverseIndexer) {
      runIntake(INTAKEREVERSESPEED);

    } else if ( shoot && flywheelAtSpeed()) {
      runIntake(INTAKEAGITATESPEED);

    } else if (manualIntake) {
      runIntake(INTAKESPEED);

    } else {
      runIntake(0);
    }
  }

  private void updateIndexer() {

    if (reverseIndexer) {
      runIndexer(INDEXERREVERSESPEED);
    } else if (shoot && flywheelAtSpeed()) {
      runIndexer(INDEXERSPEED);
    } else {
      runIndexer(0);
    }
  }

  private void updateElevator() {
    if (shoot && flywheelAtSpeed() && (! reverseIndexer)) {
      runElevator(ELEVATORSPEED);
    } else {
      runElevator(0);
    }
  }

  private void updateFlywheel() {

    setFlywheelSpeed( (flywheelOn) ? flywheelDesiredSpeed : 0 );
    SmartDashboard.putBoolean("Flywheel On", flywheelOn);
    SmartDashboard.putBoolean("Flywheel Ready", flywheelAtSpeed());

  }


  @Override
  public void periodic() {

    filteredVelocity = (filteredVelocity * 0.95) + (flywheel.getVelocity().getValueAsDouble() * 0.05);
  }

  public Command setIntake(boolean set, BallSystem ballSystem) {
    return new InstantCommand(() -> ballSystem.manualIntake = set);
  }

  public Command setShoot(boolean set, BallSystem ballSystem) {
    return new InstantCommand(() -> ballSystem.shoot = set);
  }

  public Command setFlywheel(boolean set, BallSystem ballSystem) {
    return new InstantCommand(() -> ballSystem.flywheelOn = set);
  }

  public Command setSpeed(double speed, BallSystem ballSystem) {
    return new InstantCommand(() -> ballSystem.flywheelDesiredSpeed = speed);
  }

  public Command targetSpeed(GoalTracker goalTracker, BallSystem ballSystem) {
    return new RunCommand(() -> ballSystem.flywheelDesiredSpeed = goalTracker.getFlywheelSpeed(false));
  }

  public Command updateCommand(BallSystem ballSystem) {
    return new RunCommand(() -> ballSystem.update(), this);
  }

  public Command resetBallSystem() {
    return Commands.sequence(
      new InstantCommand(() -> this.manualIntake = false),
      new InstantCommand(() -> this.shoot = false),
      new InstantCommand(() -> this.flywheelOn = false),
      new InstantCommand(() -> this.flywheelDesiredSpeed = 0),
      new InstantCommand(() -> this.update(), this)
    );
  }

}
