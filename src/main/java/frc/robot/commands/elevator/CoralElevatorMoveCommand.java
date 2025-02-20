// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This Command causes the elevator to ascend or descend
public class CoralElevatorMoveCommand extends Command {
  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port
  private CoralElevatorSubsystem m_subsystem;
  private double controllerInput=0;

  /** Right Bumper command, causes Elevator to ascend. Left Bumper command, causes Elevator to descend */
  public CoralElevatorMoveCommand() {
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
    controllerInput = controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS);
    if(Math.abs(controllerInput)<.9){
      controllerInput=0;
    }
    m_subsystem.setSpeedClimb(-Constants.ARM_SPEED*controllerInput, -Constants.ARM_SPEED*controllerInput);
    m_subsystem.setSpeedArm(-m_subsystem.getGravityControl());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}