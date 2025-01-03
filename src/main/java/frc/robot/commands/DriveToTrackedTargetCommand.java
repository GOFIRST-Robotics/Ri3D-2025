// Author: UMN Robotics Ri3D
// Last Updated: January 2023

package frc.robot.commands;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// This command rotates the robot to the best (nearest) field april tag
public class DriveToTrackedTargetCommand extends Command {

  private DriveSubsystem m_drivetrainSubsystem;
  private VisionSubsystem m_visionSubsystem;

  double angleToTarget;
  int targetTagID;
  double desiredDistanceToTarget;
  double targetArea;
  boolean usingArea;
  double translationalError;

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

  /** Rotates the robot and drives to the best (nearest) target, but instead of specifying distance
   * only specift target area.
  */
  public DriveToTrackedTargetCommand(double targetArea, boolean usingArea) {
    this(targetArea);
    this.usingArea = usingArea;
  }

  /**
   * Rotates the robot and drives to a specific april tag, but instead of specifying distance
   * only specift target area.
   * @param targetArea
   * @param targetTagID
   * @param usingArea
   */
  public DriveToTrackedTargetCommand(double targetArea, int targetTagID, boolean usingArea) {
    this(targetArea, targetTagID);
    this.usingArea = usingArea;
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
        double rotationalError = trackedTarget.getYaw();      
        double translationValue;
        if (usingArea) {
          translationalError = desiredDistanceToTarget - trackedTarget.getArea();
          translationValue = translationalError * Constants.TRACKED_TAG_AREA_DRIVE_KP;
        } else {
          translationalError = -desiredDistanceToTarget + PhotonUtils.calculateDistanceToTargetMeters(
            Constants.CAMERA_HEIGHT_METERS, 
            Constants.TARGET_HEIGHT_METERS, 
            Constants.CAMERA_PITCH_RADIANS, 
            trackedTarget.getPitch()
          );
          translationValue = translationalError * Constants.TRACKED_TAG_DISTANCE_DRIVE_KP*3;
        }
        // If the robot is too close to the target, drive backwards
        double rotationValue = -rotationalError * Constants.TRACKED_TAG_ROATION_KP;
        double leftPower =  translationValue - rotationValue; // NEGATIVE
        double rightPower = translationValue + rotationValue; // POSITVE
        double leftDriveRate;
        double rightDriveRate;
        if (leftPower > Constants.APRILTAG_POWER_CAP || rightPower > Constants.APRILTAG_POWER_CAP) {
          double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
          leftDriveRate = Math.copySign(leftPower/max, leftPower);
          rightDriveRate = Math.copySign(rightPower/max, rightPower);
        } else if (leftPower < -Constants.APRILTAG_POWER_CAP || rightPower < -Constants.APRILTAG_POWER_CAP) {
            double min = Math.max(Math.abs(leftPower), Math.abs(rightPower));
            leftDriveRate = Math.copySign(leftPower/min, leftPower);
            rightDriveRate = Math.copySign(rightPower/min, rightPower);
        } else {
          leftDriveRate = leftPower;
          rightDriveRate = rightPower;
        }
        m_drivetrainSubsystem.drive(leftDriveRate, rightDriveRate); // Finnally drive with the values we have

        // Print out all the variables for debugging
        System.out.println("Target Area: " + desiredDistanceToTarget);
        System.out.println("Current Area: " + translationValue);
        System.out.println("Distance: " + desiredDistanceToTarget);
        System.out.println("Rotational Error: " + rotationalError);
        System.out.println("Translational Error: " + translationalError);
        System.out.println("Rotational Value: " + rotationValue);
        System.out.println("Translational Value: " + translationValue);
        System.out.println("leftDriveRate: " + leftDriveRate);
        System.out.println("rightDriveRate: " + rightDriveRate); 
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
    if(usingArea) {
      return desiredDistanceToTarget <= 0.5;
    } else {
      return translationalError <= 0.2;
    }
  }
}