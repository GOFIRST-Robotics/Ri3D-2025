// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorArmSubsystem;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This command activates the Score Mid Elevator/End Effector preset
public class CoralElevatorScoreMidCommand extends Command {
  private CoralElevatorSubsystem m_subsystem_Elevator;
  private CoralElevatorArmSubsystem m_subsystem_Arm;

  /** D Pad up command, sets height to Neutral and End Effector  to Intake */
  public CoralElevatorScoreMidCommand() {
    m_subsystem_Elevator = Robot.m_CoralElevatorSubsystem;
    m_subsystem_Arm = Robot.m_CoralElevatorArmSubsystem;
    addRequirements(m_subsystem_Elevator,m_subsystem_Arm);
  }

  // Called once when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem_Elevator.climbMidGoal();
    m_subsystem_Arm.armDrop();
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