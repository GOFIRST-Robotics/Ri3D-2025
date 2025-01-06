// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This Command causes the elevator to ascend or descend
public class CoralElevatorMoveCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;
  private boolean direction;

  /** Right Bumper command, causes Elevator to ascend. Left Bumper command, causes Elevator to descend */
  public CoralElevatorMoveCommand(boolean direction) {
    this.direction = direction;
    m_subsystem = Robot.m_CoralElevatorSubsystem;
    addRequirements(m_subsystem);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    if ((m_subsystem.getPositionClimbOne() <= m_subsystem.climb_max_1) && (m_subsystem.getPositionClimbOne() >= m_subsystem.climb_min_1)) {
      m_subsystem.setSpeedClimbOne(this.direction ? -1 * Constants.ELEVATOR_SPEED : Constants.ELEVATOR_SPEED);
    } else {
      m_subsystem.setSpeedClimbOne(0);
    }
    if ((m_subsystem.getPositionClimbTwo() <= m_subsystem.climb_max_2) && (m_subsystem.getPositionClimbTwo() >= m_subsystem.climb_min_2)) {
      m_subsystem.setSpeedClimbTwo(this.direction ? -1 * Constants.ELEVATOR_SPEED : Constants.ELEVATOR_SPEED);
    } else {
      m_subsystem.setSpeedClimbTwo(0);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((m_subsystem.getPositionClimbOne() > m_subsystem.climb_max_1) || (m_subsystem.getPositionClimbOne() < m_subsystem.climb_min_1)) {
      m_subsystem.setSpeedClimbOne(0);
    }
    if ((m_subsystem.getPositionClimbTwo() > m_subsystem.climb_max_2) || (m_subsystem.getPositionClimbTwo() < m_subsystem.climb_min_2)) {
      m_subsystem.setSpeedClimbTwo(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setSpeedClimbOne(0);
    m_subsystem.setSpeedClimbTwo(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish (we don't want it to)
  }
}