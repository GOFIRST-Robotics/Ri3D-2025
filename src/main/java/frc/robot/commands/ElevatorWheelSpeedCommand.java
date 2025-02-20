// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.CoralWheelSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ElevatorWheelSpeedCommand extends InstantCommand {
  private CoralWheelSubsystem m_Subsystem;
  private double speed;

  public ElevatorWheelSpeedCommand(double speed) {
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    m_Subsystem = Robot.m_CoralWheelSubsystem;
    addRequirements(m_Subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Subsystem.setSpeedWheel(speed);
  }
}
