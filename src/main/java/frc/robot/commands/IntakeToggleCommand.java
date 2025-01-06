// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;

import java.lang.ModuleLayer.Controller;
import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeSubsystem;



public class IntakeToggleCommand extends Command {
  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port
  private IntakeSubsystem m_intakeSubsystem;
  boolean reverse;


  /** Creates a new IntakeCommand. */
  public IntakeToggleCommand(boolean reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intakeSubsystem = Robot.m_intakeSubsystem;
    addRequirements(m_intakeSubsystem);
    this.reverse = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.presetToggle();
    

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(m_intakeSubsystem.getPreset()) {
      case 0:
        new IntakeSetPosition(Constants.POSITION_ZERO);
        break;
      case 1:
        new IntakeSetPosition(Constants.POSITION_ONE);
        break;
      case 2:
        new IntakeSetPosition(Constants.POSITION_TWO);
        break;
    }
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
