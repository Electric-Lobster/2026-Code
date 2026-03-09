// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static frc.robot.Constants.ShooterConstants.ELEVATOR;
import static frc.robot.Constants.ShooterConstants.FLYWHEEL;
import static frc.robot.Constants.ShooterConstants.INDEXER;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  SparkMax indexer = new SparkMax(INDEXER, MotorType.kBrushless);
  SparkMax elevator = new SparkMax(ELEVATOR, MotorType.kBrushed);
  TalonFX flywheel = new TalonFX(FLYWHEEL);

  /** Creates a new Shooter. */
  public Shooter() {

    SparkMaxConfig indexerConfig = new SparkMaxConfig();
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    
    indexerConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    indexerConfig.smartCurrentLimit(10, 10, 10).inverted(false).idleMode(IdleMode.kBrake);

    elevatorConfig.smartCurrentLimit(10, 10, 10).inverted(false).idleMode(IdleMode.kBrake);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
