// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This is a custom Set Position command for the Climb motor
public class CoralElevatorSetPositionClimbCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;
  private double position;
  private double error;
  private double kP = 0.1; // TODO: Tune this
  private double goalThreshold = 3; // TODO: Tune this

  /** causes Climb motor to move to given position */
  public CoralElevatorSetPositionClimbCommand(double position) {
    this.position = position;
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
    this.error = position - m_subsystem.getPositionClimb();
    double output = kP * error;
    if (Math.abs(output) > 0.75) { // Max power we want to allow // TODO: Tune this
      output = Math.copySign(0.75, output);
    }
    if (Math.abs(output) < 0.05) { // Min power we want to allow // TODO: Tune this
      output = Math.copySign(0.05, output);
    }
    m_subsystem.setSpeedClimb(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setSpeedClimb(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(this.error) < this.goalThreshold;
  }
}