// Author: UMN Robotics Ri3D
// Last Updated: January 2023

package frc.robot.commands.autonomous;

import frc.robot.commands.TimedGyroDriveStraightCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is our most basic autonomous routine. It drives forward for 2 seconds, turns around 180 degrees, and then drives back for 2 seconds. */
public class DriveBack extends SequentialCommandGroup {

  // List commands here sequentially
  public DriveBack() { // List commands here sequentially
    addCommands(new TimedGyroDriveStraightCommand(2, -0.2));
  }
}