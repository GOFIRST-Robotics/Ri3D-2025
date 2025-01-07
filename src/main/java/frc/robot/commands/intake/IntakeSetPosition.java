// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSetPosition extends Command {

  IntakeSubsystem intake = Robot.m_intakeSubsystem;
  private double goal;
  private double error;
  private double kP = 0.015;

  /** Creates a new IntakeSetPosition. */
  public IntakeSetPosition(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
    goal = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // -
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.error = goal - intake.getPosition();
    double output = kP * error;
    if (Math.abs(output) > 0.1) { // Maximum power we want to allow
      output = Math.copySign(0.1, output);
    }
    if (Math.abs(output) < 0.05) { // Minimum power we want to allow
      output = Math.copySign(0.05, output);
    }
    intake.deployIntake(output-intake.getIntakeGravityControl());
    SmartDashboard.putNumber("OUTPUT", output);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) <= 0.5;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Stop the motor
    intake.deployIntake(0);
  }
}
