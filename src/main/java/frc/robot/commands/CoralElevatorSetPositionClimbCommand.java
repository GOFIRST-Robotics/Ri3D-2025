// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This is a custom Set Position command for the Climb motor
public class CoralElevatorSetPositionClimbCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;
  private double position_1;
  private double error_1;
  private double position_2;
  private double error_2;
  private double kP = 0.1; // TODO: Tune this
  private double goalThreshold = 3; // TODO: Tune this

  /** causes Climb motor to move to given position */
  public CoralElevatorSetPositionClimbCommand(double position_1, double position_2) {
    this.position_1 = position_1;
    this.position_2 = position_2;
    m_subsystem = Robot.m_CoralElevatorSubsystem;
    addRequirements(m_subsystem);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    if ((position_1 > m_subsystem.climb_max_1) || (position_1 < m_subsystem.climb_min_1) || (position_2 > m_subsystem.climb_max_2) || (position_2 > m_subsystem.climb_min_2)) {
      end(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.error_1 = position_1 - m_subsystem.getPositionClimbOne();
    double output_1 = kP * error_1;
    this.error_2 = position_2 - m_subsystem.getPositionClimbOne();
    double output_2 = kP * error_2;

    if (Math.abs(output_1) > 0.75) { // Max power we want to allow // TODO: Tune this
      output_1 = Math.copySign(0.75, output_1);
    }
    if (Math.abs(output_1) < 0.05) { // Min power we want to allow // TODO: Tune this
      output_1 = Math.copySign(0.05, output_1);
    }

    if (Math.abs(output_2) > 0.75) { // Max power we want to allow // TODO: Tune this
      output_2 = Math.copySign(0.75, output_2);
    }
    if (Math.abs(output_2) < 0.05) { // Min power we want to allow // TODO: Tune this
      output_2 = Math.copySign(0.05, output_2);
    }

    m_subsystem.setSpeedClimbOne(output_2);
    m_subsystem.setSpeedClimbOne(output_1);

    if ((position_1 > m_subsystem.climb_max_1) || (position_1 < m_subsystem.climb_min_1) || (position_2 > m_subsystem.climb_max_2) || (position_2 > m_subsystem.climb_min_2)) {
      end(true);
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
    return (Math.abs(this.error_1) < this.goalThreshold) && (Math.abs(this.error_2) < this.goalThreshold);
  }
}