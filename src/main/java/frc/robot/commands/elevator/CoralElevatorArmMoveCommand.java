// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralElevatorSubsystem;

public class CoralElevatorArmMoveCommand extends Command {
  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port
  private CoralElevatorSubsystem m_Subsystem;

  /** Creates a new CoralElevatorArmMoveCommand. */
  public CoralElevatorArmMoveCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Subsystem = Robot.m_CoralElevatorSubsystem;
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Subsystem.setSpeedArm(Constants.ARM_SPEED*controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS)-m_Subsystem.getGravityControl());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}