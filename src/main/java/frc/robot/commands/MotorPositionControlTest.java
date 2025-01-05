// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;

public class MotorPositionControlTest extends Command {

  private double error;
  private double kP = 0.1;
  private double positionGoal = 100;
  private double goalThreshold = 5;
  private SparkMax motor = new SparkMax(4, MotorType.kBrushless);

  /** Creates a new MotorPositionControlTest. */
  public MotorPositionControlTest() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(false).idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.error = positionGoal - motor.getEncoder().getPosition();
    double output = kP * error;
    if (Math.abs(output) > 0.5) { // Maximum power we want to allow
      output = Math.copySign(0.75, output);
    }
    if (Math.abs(output) < 0.05) { // Minimum power we want to allow
      output = Math.copySign(0.15, output);
    }
    motor.set(output);
    System.out.println(motor.getEncoder().getPosition());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.error) < this.goalThreshold;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motor
    motor.set(0);
  }
}
