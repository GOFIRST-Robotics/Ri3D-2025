// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeAlgaeFromFloorCommand extends Command {
  boolean isFinished;
  double previousCurrent;
  IntakeSubsystem m_intakeSubsystem;
  /** Creates a new IntakeAlgaeFromFloorCommand. */
  public IntakeAlgaeFromFloorCommand() {
    m_intakeSubsystem = Robot.m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    previousCurrent = 9999.0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    (new IntakeCommand(false)).schedule();
    (new IntakeToggleCommand(Constants.PICK_UP_ALGAE_ID)).schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_intakeSubsystem.getIntakeMotorCurrent() > previousCurrent+2*m_intakeSubsystem.getIntakeMotorCurrentStandardDeviation()) {
      (new IntakeSetPosition(Constants.HOLD_ALGAE_ID)).schedule();
      isFinished = true;
    }
    previousCurrent = m_intakeSubsystem.getIntakeMotorCurrent();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    (new IntakeSetPosition(Constants.HOLD_ALGAE_ID)).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
