// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This command causes the Coral held by the End Effector to be dropped
public class CoralElevatorDropCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;

  /** D Pad left command, causes End Effector to drop held Coral */
  public CoralElevatorDropCommand() {
    m_subsystem = Robot.m_CoralElevatorSubsystem;
    addRequirements(m_subsystem);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    (new CoralElevatorSetPositionLowerCommand(0.25)).schedule();
    m_subsystem.setAngleFinger(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // -
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    (new CoralElevatorSetPositionLowerCommand(0)).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish (we don't want it to)
  }
}