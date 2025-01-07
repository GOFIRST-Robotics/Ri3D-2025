// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands.autonomous.example_basic_auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroStraight;
import frc.robot.commands.autonomous.basic_path_planning.Drivetrain_GyroTurn;

// This autonomous routine drives in a square!
public class SquareAutonomous extends SequentialCommandGroup{
    public SquareAutonomous(){
    addCommands(new Drivetrain_GyroStraight(0.5, 0.2)); // Drive straight
    addCommands(new Drivetrain_GyroTurn(90)); // Turn 90 degrees
    addCommands(new Drivetrain_GyroStraight(0.5, 0.2)); // Drive straight
    addCommands(new Drivetrain_GyroTurn(90)); // Turn 90 degrees
    addCommands(new Drivetrain_GyroStraight(0.5, 0.2)); // Drive straight
    addCommands(new Drivetrain_GyroTurn(90)); // Turn 90 degrees
    addCommands(new Drivetrain_GyroStraight(0.5, 0.2)); // Drive straight
    addCommands(new Drivetrain_GyroTurn(90)); // Turn 90 degrees
    }
}