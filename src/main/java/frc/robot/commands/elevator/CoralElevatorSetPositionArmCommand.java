// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This is a custom Set Position command for the Arm motor
public class CoralElevatorSetPositionArmCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;
  private double position;
  private double error;
  private double kP = 0.1; // TODO: Tune this

  /** causes Arm motor to move to given position */
  public CoralElevatorSetPositionArmCommand(double position) {
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
    this.error = position - m_subsystem.getPositionArm();
    double output = kP * error;

    if(error > m_subsystem.getPositionArm()) { // Gravity compensator
      output += Constants.GRAVITY_CONST;
    } else if (error < m_subsystem.getPositionArm()){
      output -= Constants.GRAVITY_CONST;
    }

    if (Math.abs(output) > 0.75) { // Max power we want to allow // TODO: Tune this
      output = Math.copySign(0.75, output);
    }

    if (Math.abs(output) < 0.05) { // Min power we want to allow // TODO: Tune this
      output = Math.copySign(0.05, output);
    }

    m_subsystem.setSpeedArm(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish (we don't want it to unless interrupted)
  }
}