// Author: UMN Robotics Ri3D
// Last Updated: January 2025

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    PhotonPoseEstimator photonPoseEstimator;
    List<PhotonPipelineResult> results; // Stores all the data that Photonvision returns
    PhotonPipelineResult result; // Stores the latest data that Photonvision returns
    boolean hasTarget; // Stores whether or not a target is detected
    Matrix<N3, N1> currentStdDevs;
    Transform3d robotToCam;

    public VisionSubsystem() {
        robotToCam = new Transform3d(
            new Translation3d(Constants.CAMERA_FORWARD_METERS, Constants.CAMERA_HORIZONTAL_METERS, Constants.CAMERA_HEIGHT_METERS), 
            new Rotation3d(0, Constants.CAMERA_PITCH_RADIANS,0)
        );
        photonPoseEstimator = new PhotonPoseEstimator(Constants.FIELD_APRILTAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);
        camera = new PhotonCamera(Constants.USB_CAMERA_NAME); // Declare the name of the camera used in the pipeline
    }

    @Override
    public void periodic() {
        results = camera.getAllUnreadResults(); // Query the all unread result from PhotonVision
        if (!results.isEmpty()) {
            result = results.get(results.size()-1); // Get the latest result from the list of PhotonVision results
            hasTarget = result.hasTargets(); // If the camera has detected an apriltag target, the hasTarget boolean will be true
        }
        SmartDashboard.putBoolean("HasTarget", hasTarget);
    }
    
    public PhotonTrackedTarget getTargetWithID(int id) { // Returns the apriltag target with the specified ID (if it exists)
        List<PhotonTrackedTarget> targets = result.getTargets(); // Create a list of all currently tracked targets
        for (PhotonTrackedTarget i : targets) {
            if (i.getFiducialId() == id) { // Check the ID of each target in the list
                return i; // Found the target with the specified ID!
            }
        }
        return null; // Failed to find the target with the specified ID
    }
    
    public PhotonTrackedTarget getBestTarget() {
        if (hasTarget) {
            return result.getBestTarget(); // Returns the best (closest) target
        }
        else {
            return null; // Otherwise, returns null if no targets are currently found
        }
    }

    public boolean getHasTarget() {
        return hasTarget; // Returns whether or not a target was found
    }

    public double getDistanceToTarget(PhotonTrackedTarget target) {
        if (!hasTarget) {
            return 0;
        }
        double april_tag_pitch = target.getPitch();

        double distance = PhotonUtils.calculateDistanceToTargetMeters(
            Constants.CAMERA_HEIGHT_METERS, 
            Constants.TARGET_HEIGHT_METERS, 
            Constants.CAMERA_PITCH_RADIANS, 
            Math.toRadians(target.getPitch())
        );

        // Print the area and pitch of the target
        //System.out.println("Area: " + april_tag_height + "Pitch: " + april_tag_pitch);
        SmartDashboard.putNumber("t_distance", distance);
        SmartDashboard.putNumber("t_pitch", april_tag_pitch);
        return distance;
    }

    public boolean InRange(double distanceThreshold, double distanceThresholdRange, double angleThreshold, double angleThresholdRange) {
        if (!hasTarget) {
            return false;
        }
    
        PhotonTrackedTarget bestTarget = getBestTarget();
        double distanceToTarget  = getDistanceToTarget(bestTarget);
        double angleToTarget = bestTarget.getYaw(); // Assuming yaw gives the angle
        double skewTarget = bestTarget.getSkew();

        //boolean inRange = Math.abs(distanceToTarget) <= distanceThreshold && Math.abs(angleToTarget) <= angleThreshold;
        boolean inRange = Math.abs(Math.abs(distanceToTarget) - distanceThreshold) >= distanceThresholdRange && Math.abs(Math.abs(angleToTarget) - angleThreshold) >= angleThresholdRange;
        
        SmartDashboard.putNumber("t_distance", distanceToTarget);
        SmartDashboard.putNumber("t_angle", angleToTarget);
        SmartDashboard.putNumber("t_skew", skewTarget);
        SmartDashboard.putBoolean("InRange", inRange);
    
        return inRange;
    }
    
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> estimation = Optional.empty();
        for (PhotonPipelineResult change : results) {
            estimation = photonPoseEstimator.update(change);
            updateEstimationStdDevs(estimation, change.getTargets());
        }
        return estimation;
    }

    public Matrix<N3, N1> getEstimationStdDevs() {
        return currentStdDevs;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            currentStdDevs = Constants.SINGLE_TAG_STDDEVS;
        } else {
            // Pose present. Start running Heuristic
            Matrix<N3, N1> estStdDevs = Constants.SINGLE_TAG_STDDEVS;
            int tagCount = 0;
            double averageDistance = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (PhotonTrackedTarget target : targets) {
                Optional<Pose3d> tagPosition = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
                if (tagPosition.isEmpty()) {
                    continue;
                }
                tagCount++;
                averageDistance += tagPosition.get().toPose2d().getTranslation().getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (tagCount == 0) {
                // No tags visible. Default to single-tag std devs
                currentStdDevs = Constants.SINGLE_TAG_STDDEVS;
            } else {
                // One or more tags visible, run the full heuristic.
                averageDistance /= tagCount;
                if (tagCount > 1) { // Decrease std devs if multiple targets are visible
                    estStdDevs = Constants.MULTI_TAG_STDDEVS;
                }
                if (tagCount == 1 && averageDistance > 4) {// Increase std devs based on (average) distance
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                } else {
                    estStdDevs = estStdDevs.times(1 + (averageDistance * averageDistance / 30));
                }
                currentStdDevs = estStdDevs;
            }
        }
    }
}