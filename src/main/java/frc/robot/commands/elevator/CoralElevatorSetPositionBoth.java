// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This is a custom Set Position command for the Arm motor
public class CoralElevatorSetPositionBoth extends Command {
  private CoralElevatorSubsystem m_subsystem;
  private double position;
  private double error;
  private double kP = 0.02;

  private double position_1;
  private double error_1;
  private double position_2;
  private double error_2;

  private double kP_climb = 0.05; // TODO: Tune this
  private double goalThreshold = 3; // TODO: Tune this

  /** Causes arm and climb motor to move to given position */
  public CoralElevatorSetPositionBoth(double position, double position_1, double position_2) {
    this.position = position;
    this.position_1 = position_1;
    this.position_2 = position_2;
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
    this.error = position - m_subsystem.getPositionArm();
    double output = kP * error;
    this.error_1 = position_1 - m_subsystem.getPositionClimbOne();
    double output_1 = kP_climb * error_1;
    this.error_2 = position_2 - m_subsystem.getPositionClimbOne();
    double output_2 = kP_climb * error_2;

    if (Math.abs(output) > 0.2) { // Max power we want to allow // TODO: Tune this
      output = Math.copySign(0.2, output);
    }
    if (Math.abs(output) < 0.05) { // Min power we want to allow // TODO: Tune this
      output = Math.copySign(0.05, output);
    }

    if (Math.abs(error) > 0.5) {
      m_subsystem.setSpeedArm(output-m_subsystem.getGravityControl());
    } else {
      m_subsystem.setSpeedArm(-m_subsystem.getGravityControl());
    }

    if (Math.abs(output_1) > 0.2) { // Max power we want to allow // TODO: Tune this
      output_1 = Math.copySign(0.2, output_1);
    }
    if ((Math.abs(this.error_1) > this.goalThreshold) && Math.abs(output_1) < 0.05) { // Min power we want to allow // TODO: Tune this
      output_1 = Math.copySign(0.05, output_1);
    }
    if (Math.abs(output_2) > 0.2) { // Max power we want to allow // TODO: Tune this
      output_2 = Math.copySign(0.2, output_2);
    }
    if (Math.abs(this.error_2) > this.goalThreshold && Math.abs(output_2) < 0.05) { // Min power we want to allow // TODO: Tune this
      output_2 = Math.copySign(0.05, output_2);
    }

    m_subsystem.setSpeedClimbOne(output_1);
    m_subsystem.setSpeedClimbTwo(output_2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopArm();
    m_subsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(error) <= 0.5 && (Math.abs(this.error_1) < this.goalThreshold) && (Math.abs(this.error_2) < this.goalThreshold);
  }
}