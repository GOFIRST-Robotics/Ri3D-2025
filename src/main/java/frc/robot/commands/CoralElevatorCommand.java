// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This command takes the joystick inputs and demands that the drivetrain follow them
public class CoralElevatorCommand extends Command {
  private CoralElevatorSubsystem m_subsystem;
  // TODO impliment modular extension system.
  // private int mode;

  /** Default drive command that takes the joystick inputs and demands that the drivetrain follow them */
  public CoralElevatorCommand() {
    m_subsystem = Robot.m_CoralElevatorSubsystem;
    addRequirements(m_subsystem);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    // mode system switching goes here
    m_subsystem.setPositionClimb(936);
    m_subsystem.setAngle(90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // -
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Command will never finish (we don't want it to)
  }
}