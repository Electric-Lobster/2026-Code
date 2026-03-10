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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallSystem extends SubsystemBase {

  //MOTORS
  SparkMax intake = new SparkMax(INTAKE, MotorType.kBrushless);
  SparkMax indexer = new SparkMax(INDEXER, MotorType.kBrushless);
  SparkMax elevator = new SparkMax(ELEVATOR, MotorType.kBrushed);
  TalonFX flywheel = new TalonFX(FLYWHEEL);

  //Flywheel Control
  PIDController flywheelPID = new PIDController(0.5, 1, 0);
  SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(1.5, 0.1);

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


  public boolean setFlywheelSpeed(double speed) {

    double pidVal = flywheelPID.calculate(flywheel.getVelocity().getValueAsDouble(), speed);
    double feedfowardVal = flywheelFeedforward.calculateWithVelocities(flywheel.getVelocity().getValueAsDouble(), speed);

    runFlywheel(pidVal + feedfowardVal);

    return (speed - flywheel.getVelocity().getValueAsDouble()) < 2;
  }


  public void update() {

    updateIntake();

    updateIndexer();

    updateElevator();

    updateFlywheel();

  }

  private void updateIntake() {}

  private void updateIndexer() {}

  private void updateElevator() {}

  private void updateFlywheel() {}

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
