// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.ShooterConstants.ELEVATOR;
import static frc.robot.Constants.ShooterConstants.ELEVATORCURRANTLIMITS;
import static frc.robot.Constants.ShooterConstants.ELEVATORINVERT;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL;
import static frc.robot.Constants.ShooterConstants.FLYWHEELINVERT;
import static frc.robot.Constants.ShooterConstants.INDEXER;
import static frc.robot.Constants.ShooterConstants.INDEXERCURRANTLIMITS;
import static frc.robot.Constants.ShooterConstants.INDEXERINVERT;

import java.net.PasswordAuthentication;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  //MOTORS
  SparkMax indexer = new SparkMax(INDEXER, MotorType.kBrushless);
  SparkMax elevator = new SparkMax(ELEVATOR, MotorType.kBrushed);
  TalonFX flywheel = new TalonFX(FLYWHEEL);

  /** Creates a new Shooter. */
  public Shooter() {

    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    indexerConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    indexerConfig.smartCurrentLimit(INDEXERCURRANTLIMITS[0], INDEXERCURRANTLIMITS[1], INDEXERCURRANTLIMITS[2]).inverted(INDEXERINVERT).idleMode(IdleMode.kBrake);

    elevatorConfig.smartCurrentLimit(ELEVATORCURRANTLIMITS[0], ELEVATORCURRANTLIMITS[1], ELEVATORCURRANTLIMITS[2]).inverted(ELEVATORINVERT).idleMode(IdleMode.kBrake);

    indexer.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
    flywheel.getConfigurator().apply( new MotorOutputConfigs().withInverted(FLYWHEELINVERT).withNeutralMode(NeutralModeValue.Brake));
  }

  public void runIndexer(double throttle) {

    indexer.set(throttle);

  }

  public void runElevator(double throttle) {

    elevator.set(throttle);

  }

  public void runFlywheel(double voltage) {

    flywheel.setVoltage(voltage);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
