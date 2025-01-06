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
    // -
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.setSpeedClimb(this.direction ? -1 * Constants.ELEVATOR_SPEED : Constants.ELEVATOR_SPEED, this.direction ? -1 * Constants.ELEVATOR_SPEED : Constants.ELEVATOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish (we don't want it to)
  }
}