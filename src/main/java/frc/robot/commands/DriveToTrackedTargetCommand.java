// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// This command rotates the robot to the best (nearest) field april tag
public class DriveToTrackedTargetCommand extends Command {
  private DriveSubsystem m_drivetrainSubsystem;
  private VisionSubsystem m_visionSubsystem;

  private int targetTagID;
  private double desiredDistanceToTarget;
  private double distance;

  private PhotonTrackedTarget previousTrackedTarget;

  /** Rotates the robot and drives to the best (nearest) tracked target, can be used for either 
   * april tags or retroreflective tape tracked by photonvision
  */
  public DriveToTrackedTargetCommand(double distanceToTarget) {
    m_drivetrainSubsystem = Robot.m_driveSubsystem;
    m_visionSubsystem = Robot.m_visionSubsystem;
    desiredDistanceToTarget = distanceToTarget;
    distance = desiredDistanceToTarget + 1;
    addRequirements(m_drivetrainSubsystem, m_visionSubsystem);
  }

  /** Rotates the robot and drives to a specific april tag*/
  public DriveToTrackedTargetCommand(double distanceToTarget, int targetTagID) {
    this(distanceToTarget);
    this.targetTagID = targetTagID; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Robot.manualDriveControl = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {      
    SmartDashboard.putNumber("AprilTag Distance", distance);

    PhotonTrackedTarget trackedTarget;

    if (!m_visionSubsystem.getHasTarget()) {
      trackedTarget = previousTrackedTarget;
    }

    if(targetTagID == 0) {
      // targetTagID is 0 if an ID is not provided, 0 is a safe bet as it would be a blank square.
      trackedTarget = m_visionSubsystem.getBestTarget();
    } else {
      trackedTarget = m_visionSubsystem.getTargetWithID(targetTagID);
    }

    if (trackedTarget != null) { // If a valid target has been retrieved
      // A bunch of math to tell the drivetrain how to drive to the target 
      // while turning at the same time, till it is a certian distance away.
      previousTrackedTarget = trackedTarget;
      Transform3d targetRelativeLocation = trackedTarget.getBestCameraToTarget(); // Get the apriltag's relative location baised on the camera's location

      double rotationalError = targetRelativeLocation.getRotation().getZ();
      double forwardError = targetRelativeLocation.getX() - desiredDistanceToTarget; 
      double leftError = targetRelativeLocation.getY(); 

      distance = Math.sqrt(Math.pow(leftError, 2) + Math.pow(forwardError, 2)); // Stores the distance

      // Calcualte powers of driving to fix error
      double rotationValue = (Math.copySign(Math.PI, rotationalError) - rotationalError) * Constants.TRACKED_TAG_ROATION_KP;
      double forwardValue = forwardError * Constants.TRACKED_TAG_FORWARD_DRIVE_KP;
      double leftValue = -leftError * Constants.TRACKED_TAG_STRAFE_DRIVE_KP;
      
      // Limit the power of the drive rate directions 
      double rotationDriveRate = limit(rotationValue, Constants.APRILTAG_ROTATION_POWER_CAP);
      double forwardDriveRate = limit(forwardValue, Constants.APRILTAG_FORWARD_POWER_CAP);
      double strafeDriveRate = limit(leftValue, Constants.APRILTAG_STRAFE_POWER_CAP);

      // m_drivetrainSubsystem.driveCartesian(forwardDriveRate, strafeDriveRate, rotationDriveRate);
      m_drivetrainSubsystem.driveCartesian(forwardDriveRate, strafeDriveRate, rotationDriveRate);

      // Print out all the variables for debugging
      // System.out.println("Distance: " + desiredDistanceToTarget);
      // System.out.println("Rotational Error: " + rotationalError);
      // System.out.println("Forward Error: " + forwardError);
      // System.out.println("Left Error: " + leftError);
      // System.out.println("Rotational Value: " + rotationValue);
      // System.out.println("Forward Value: " + forwardValue);
      // System.out.println("Left Value: " + leftValue);
      // System.out.println("forwardDriveRate: " + forwardDriveRate);
      // System.out.println("strafeDriveRate: " + strafeDriveRate); 
      // System.out.println("rotationDriveRate: " + rotationDriveRate); 

      // Use Smartdashboard for Debugging
      // SmartDashboard.putNumber("Forward Value", forwardValue);
      // SmartDashboard.putNumber("Left Value", leftValue);
      // SmartDashboard.putNumber("Rotation Value", rotationValue);
      // SmartDashboard.putNumber("forwardDriveRate", forwardDriveRate);
      // SmartDashboard.putNumber("strafeDriveRate", strafeDriveRate);
      // SmartDashboard.putNumber("rotationDriveRate", rotationDriveRate);
      // SmartDashboard.putNumber("AprilTag Z Angle", rotationalError);
    } else {
      m_drivetrainSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.manualDriveControl = true;
    m_drivetrainSubsystem.stop(); // Stop the drivetrain motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return distance <= Constants.APRILTAG_TRACKING_DISTANCE_THRESHOLD;
  }

  public double limit(double value, double limit) {
    return Math.copySign(Math.min(Math.abs(value), limit), value);
  }
}