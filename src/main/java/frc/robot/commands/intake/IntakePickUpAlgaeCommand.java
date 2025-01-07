// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePickUpAlgaeCommand extends Command {

  private IntakeSubsystem m_intakeSubsystem;

  private double goalPosition = Constants.PICK_UP_ALGAE_POSITION;
  private double positionError;
  private double kP = Constants.INTAKE_ARM_kP;

  /** Creates a new IntakePickUpAlgaeCommand. */
  public IntakePickUpAlgaeCommand() {
    m_intakeSubsystem = Robot.m_intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intakeSubsystem.setIntakeBarPower(Constants.INTAKE_BAR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.positionError = goalPosition - m_intakeSubsystem.getIntakeArmPosition();

    double positionValue = kP * positionError;
    double power = Math.copySign(Math.min(Math.max(Math.abs(positionValue), Constants.INTAKE_ARM_MIN_POWER), Constants.INTAKE_ARM_MAX_POWER), positionValue); // Limit the power

    m_intakeSubsystem.setIntakeArmPower(power - m_intakeSubsystem.getIntakeGravityControl());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.stopIntakeBar();
    m_intakeSubsystem.stopIntakeArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
