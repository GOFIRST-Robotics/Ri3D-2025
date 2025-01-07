// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This Command causes the Arm to tilt up or down
public class CoralElevatorArmMoveCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;
  private boolean direction;

  /** Start command, causes Arm to tilt up. Prev command, causes Arm to tilt down */
  public CoralElevatorArmMoveCommand(boolean direction) {
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
    m_subsystem.setSpeedArm(this.direction ? -1 * Constants.ARM_SPEED : Constants.ARM_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish (we don't want it to)
  }
}