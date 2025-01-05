// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
    if (m_subsystem.getAngleFinger() == 0) {
      m_subsystem.setAngleFinger(90);
    }
    if (m_subsystem.getAngleFinger() == 90) {
      m_subsystem.setAngleFinger(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // -
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // -
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish (we don't want it to)
  }
}