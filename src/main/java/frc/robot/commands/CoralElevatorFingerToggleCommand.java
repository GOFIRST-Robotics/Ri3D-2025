// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This command toggles the Finger into and out of the disengaged position
public class CoralElevatorFingerToggleCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;

  /** B button, toggles Finger */
  public CoralElevatorFingerToggleCommand() {
    m_subsystem = Robot.m_CoralElevatorSubsystem;
    addRequirements(m_subsystem);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setAngleFinger(m_subsystem.fingerState ?  90.0 : 0.0);
    m_subsystem.fingerState = !m_subsystem.fingerState;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Finger State: " + m_subsystem.fingerState);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; // Command will finish right away
  }
}