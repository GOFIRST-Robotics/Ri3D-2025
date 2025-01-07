// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakePickUpAlgaeCommand extends Command {
  boolean isFinished;
  double previousCurrent;
  IntakeSubsystem m_intakeSubsystem;
  /** Creates a new IntakePickUpAlgaeCommand. */
  public IntakePickUpAlgaeCommand() {
    m_intakeSubsystem = Robot.m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    previousCurrent = 9999.0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    (new IntakeSetBarPowerCommand(Constants.INTAKE_BAR_SPEED)).schedule();
    (new IntakeSetArmPositionCommand(Constants.PICK_UP_ALGAE_POSITION)).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intakeSubsystem.getIntakeCurrentSum() > previousCurrent) {
      (new IntakeSetArmPositionCommand(Constants.HOLD_ALGAE_POSITION)).schedule();
      isFinished = true;
    }
    previousCurrent = m_intakeSubsystem.getIntakeCurrentSum();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    (new IntakeSetArmPositionCommand(Constants.HOLD_ALGAE_POSITION)).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
