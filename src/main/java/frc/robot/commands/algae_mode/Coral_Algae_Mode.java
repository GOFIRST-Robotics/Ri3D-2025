// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.algae_mode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
// import frc.robot.commands.elevator.CoralElevatorWheelMoveCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Coral_Algae_Mode extends InstantCommand {
  public Coral_Algae_Mode() {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(controller);    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // new Trigger(() -> controller.getRawButton(Constants.LEFT_TRIGGER_BUTTON)).whileTrue(new IntakeSetBarPowerCommand(-Constants.INTAKE_BAR_SPEED)); // Outake algae
    // new Trigger(() -> controller.getRawButton(Constants.LEFT_TRIGGER_BUTTON)).whileTrue(new CoralElevatorWheelMoveCommand(-Constants.WHEEL_SPEED)); // Outake coral
    if(Robot.CORAL==0){
      Robot.CORAL = 8;
      Robot.ALGAE = 0;
    } else {
      Robot.CORAL = 0;
      Robot.ALGAE = 8;
    }
    
  }
}
