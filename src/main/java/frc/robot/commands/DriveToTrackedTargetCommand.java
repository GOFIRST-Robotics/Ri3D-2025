// Author: UMN Robotics Ri3D
// Last Updated: January 2023

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// This command rotates the robot to the best (nearest) field april tag
public class DriveToTrackedTargetCommand extends Command {

  private DriveSubsystem m_drivetrainSubsystem;
  private VisionSubsystem m_visionSubsystem;

  int targetTagID;
  double angleToTarget;
  double desiredDistanceToTarget;
  double distance;

  /** Rotates the robot and drives to the best (nearest) tracked target, can be used for either 
   * april tags or retroreflective tape tracked by photonvision
  */
  public DriveToTrackedTargetCommand(double distanceToTarget) {
    m_drivetrainSubsystem = Robot.m_driveSubsystem;
    m_visionSubsystem = Robot.m_visionSubsystem;
    desiredDistanceToTarget = distanceToTarget;
    addRequirements(m_drivetrainSubsystem, m_visionSubsystem);
  }

  /** Rotates the robot and drives to a specific april tag*/
  public DriveToTrackedTargetCommand(double distanceToTarget, int targetTagID) {
    this(distanceToTarget);
    this.targetTagID = targetTagID; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_visionSubsystem.getHasTarget()) {
      PhotonTrackedTarget trackedTarget;
      if(targetTagID == 0) {
        // targetTagID is 0 if an ID is not provided, 0 is a safe bet as it would be a blank square.
        trackedTarget = m_visionSubsystem.getBestTarget();
      } else {
        trackedTarget = m_visionSubsystem.getTargetWithID(targetTagID);
      }
      if (trackedTarget != null) { // If a valid target has been retrieved
        // A bunch of math to tell the drivetrain how to drive to the target 
        // while turning at the same time, till it is a certian distance away.
        Transform3d targetRelativeLocation = trackedTarget.getBestCameraToTarget(); // Get the apriltag's relative location baised on the camera's location

        double rotationalError = trackedTarget.getYaw();
        double forwardError = -desiredDistanceToTarget + targetRelativeLocation.getX(); 
        double leftError = targetRelativeLocation.getY();
        
        distance = m_visionSubsystem.getDistanceToTarget(trackedTarget); // Stores the distance 

        double rotationValue = -rotationalError * Constants.TRACKED_TAG_ROATION_KP;
        double forwardValue = -forwardError * Constants.TRACKED_TAG_DISTANCE_DRIVE_KP*3;
        double leftValue = -leftError * Constants.TRACKED_TAG_DISTANCE_DRIVE_KP;
        
        // If the robot is too close to the target, drive backwards
        double forwardDriveRate;
        double strafeDriveRate;
        double rotationDriveRate;



        // if (leftPower > Constants.APRILTAG_POWER_CAP || rightPower > Constants.APRILTAG_POWER_CAP) {
        //   double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        //   leftDriveRate = Math.copySign(leftPower/max, leftPower);
        //   rightDriveRate = Math.copySign(rightPower/max, rightPower);
        // } else if (leftPower < -Constants.APRILTAG_POWER_CAP || rightPower < -Constants.APRILTAG_POWER_CAP) {
        //     double min = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        //     leftDriveRate = Math.copySign(leftPower/min, leftPower);
        //     rightDriveRate = Math.copySign(rightPower/min, rightPower);
        // } else {
        //   leftDriveRate = leftPower;
        //   rightDriveRate = rightPower;
        // }

        // m_drivetrainSubsystem.driveCartesian(leftDriveRate, rightDriveRate, rotationValue);

        // Print out all the variables for debugging
        System.out.println("Target Area: " + desiredDistanceToTarget);
        System.out.println("Distance: " + desiredDistanceToTarget);
        System.out.println("Rotational Error: " + rotationalError);
        System.out.println("Forward Error: " + forwardError);
        System.out.println("Left Error: " + leftError);
        System.out.println("Rotational Value: " + rotationValue);
        System.out.println("Forward Value: " + forwardValue);
        System.out.println("Left Value: " + leftValue);
        // System.out.println("forwardDriveRate: " + forwardDriveRate);
        // System.out.println("strafeDriveRate: " + strafeDriveRate); 
        // System.out.println("rotationDriveRate: " + rotationDriveRate); 
      }
    } else {
      m_drivetrainSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.stop(); // Stop the drivetrain motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return distance <= 0.2;
  }
}