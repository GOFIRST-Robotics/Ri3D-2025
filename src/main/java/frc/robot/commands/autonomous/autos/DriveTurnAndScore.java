// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.ElevatorWheelSpeedCommand;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;
import frc.robot.commands.elevator.CoralElevatorSetPositionBoth;
import frc.robot.subsystems.CoralElevatorSubsystem;

/** Autonomous Mode (Default) ******************************************************
 * This basic autonomous routine drives forward 1 meter using encoder feedback */
public class DriveTurnAndScore extends SequentialCommandGroup {
  private CoralElevatorSubsystem m_subsystem;

  // List commands here sequentially
  public DriveTurnAndScore(String scorePosition, boolean left) { // List commands here sequentially
    m_subsystem = Robot.m_CoralElevatorSubsystem;
    addRequirements(m_subsystem);
    int angle = 60;
    if(!left){
        angle*=-1;
    }
    addCommands(new Drivetrain_GyroStraight(1.5748, 0.15), //65.500657in without bumper
                new Drivetrain_GyroTurn(angle),
                new ParallelCommandGroup(new Drivetrain_GyroStraight(0.8636, 0.15),new CoralElevatorSetPositionBoth(scorePosition)), //36.792600in without bumper
                new ElevatorWheelSpeedCommand(-Constants.WHEEL_SPEED),
                new WaitCommand(.5), 
                new ElevatorWheelSpeedCommand(0));
  }
}