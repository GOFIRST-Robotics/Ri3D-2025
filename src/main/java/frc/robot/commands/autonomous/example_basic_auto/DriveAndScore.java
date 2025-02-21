// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.autonomous.example_basic_auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.ElevatorWheelSpeedCommand;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.elevator.CoralElevatorSetPositionBoth;

/** Autonomous Mode (Default) ******************************************************
 * This basic autonomous routine drives forward 1 meter using encoder feedback */
public class DriveAndScore extends SequentialCommandGroup {

  // List commands here sequentially
  public DriveAndScore(String x) { // List commands here sequentially
    addCommands(new ParallelCommandGroup(new Drivetrain_GyroStraight(1.4478, 0.15),new CoralElevatorSetPositionBoth(x)),
                new ElevatorWheelSpeedCommand(-Constants.WHEEL_SPEED),
                new WaitCommand(.5),
                new ElevatorWheelSpeedCommand(0));
  }
}