// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.CoralElevatorArmSubsystem;
import frc.robot.subsystems.CoralElevatorSubsystem;

// This Command causes the elevator to ascend or descend
public class CoralElevatorMoveCommand extends Command {
  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port
  private CoralElevatorSubsystem m_subsystem_Elevator;
  // private CoralElevatorArmSubsystem m_subsystem_Arm;
  private double controllerInput=0;

  /** Right Bumper command, causes Elevator to ascend. Left Bumper command, causes Elevator to descend */
  public CoralElevatorMoveCommand() {
    m_subsystem_Elevator = Robot.m_CoralElevatorSubsystem;
    // m_subsystem_Arm = Robot.m_CoralElevatorArmSubsystem;
    addRequirements(m_subsystem_Elevator);
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
    m_subsystem_Elevator.setSpeedClimb(-Constants.ARM_SPEED*controllerInput, -Constants.ARM_SPEED*controllerInput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem_Elevator.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}