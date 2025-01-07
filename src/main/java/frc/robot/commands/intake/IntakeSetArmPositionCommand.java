// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeSetArmPositionCommand extends Command {
  IntakeSubsystem m_IntakeSubsystem = Robot.m_intakeSubsystem;
  private double goalPosition;
  private double positionError;
  private double kP = Constants.INTAKE_ARM_kP;

  /** Creates a new IntakeSetPosition. */
  public IntakeSetArmPositionCommand(double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem);
    goalPosition = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.positionError = goalPosition - m_IntakeSubsystem.getIntakeArmPosition();
    double output = kP * positionError;
    if (Math.abs(output) > 0.25) { // Maximum power we want to allow
      output = Math.copySign(0.25, output);
    }
    if (Math.abs(output) < 0.05) { // Minimum power we want to allow
      output = Math.copySign(0.05, output);
    }
    m_IntakeSubsystem.setIntakeArmPower(output-m_IntakeSubsystem.getIntakeGravityControl());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(positionError) <= 0.5;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.stopIntakeArm();
  }
}
