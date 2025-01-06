// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This command activates the Neutral Elevator/End Effector preset
public class CoralElevatorNeutralCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;

  /** D Pad down command, sets height to Neutral and End Effector  to Intake */
  public CoralElevatorNeutralCommand() {
    m_subsystem = Robot.m_CoralElevatorSubsystem;
    addRequirements(m_subsystem);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.lowerPlayerIntake();
    m_subsystem.climbNeutral();
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
    return true; // Command will finish immediately
  }
}