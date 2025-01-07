// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeReverseCommand extends Command {
  private IntakeSubsystem m_IntakeSubsystem;

  private boolean reverse;

  /** Creates a new IntakeCommand. */
  public IntakeReverseCommand(boolean reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_IntakeSubsystem = Robot.m_intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
    this.reverse = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.setIntakeBarPower(this.reverse ? -1 * Constants.INTAKE_BAR_SPEED : Constants.INTAKE_BAR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stopIntakeBar();
  }
}
