// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralWheelSubsystem extends SubsystemBase {
  private SparkMax wheelMotor; // NEO motor

  /** Creates a new CoralWheelSubsystem. */
  public CoralWheelSubsystem() {
    wheelMotor = new SparkMax(Constants.END_EFFECTOR_WHEEL_MOTOR_ID, MotorType.kBrushless);
    configureSparkMAX(wheelMotor, Constants.ELEVATOR_WHEEL_INVERT);
  }

  public void setSpeedWheel(double speed) {
    // Spark Max set() method
    wheelMotor.set(speed);
  }

  private void configureSparkMAX(SparkMax max, boolean reverse) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(reverse).idleMode(IdleMode.kBrake);
    max.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
    

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
