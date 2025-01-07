// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeToggleCommand extends Command {
  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port
  private IntakeSubsystem m_intakeSubsystem;
  int preset;

  /** Creates a new IntakeCommand. */
  public IntakeToggleCommand(int preset) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = Robot.m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    this.preset = preset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setPreset(preset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_intakeSubsystem.getPreset()) {
      case Constants.HOLD_ALGAE_ID:
        (new IntakeSetPosition(Constants.HOLD_ALGAE)).schedule();
        break;
      case Constants.HOLD_CORAL_ID:
        (new IntakeSetPosition(Constants.HOLD_CORAL)).schedule();;
        break;
      case Constants.PICK_UP_ALGAE_ID:
        (new IntakeSetPosition(Constants.PICK_UP_ALGAE)).schedule();;
        break;
        case Constants.PICK_UP_CORAL_ID:
        (new IntakeSetPosition(Constants.PICK_UP_CORAL)).schedule();;
        break;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}