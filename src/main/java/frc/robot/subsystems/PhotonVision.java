package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues.PhotonVisionConstants;

/**
 * The PhotonVision subsystem handles vision processing using PhotonVision cameras.
 */
public class Photonvision extends SubsystemBase {
  // PhotonVision cameras
  PhotonCamera camera1 = new PhotonCamera("Camera One");
  PhotonCamera camera2 = new PhotonCamera("Camera Two");

  // Tracked targets from the cameras
  PhotonTrackedTarget target1;
  PhotonTrackedTarget target2;

  // Pose estimator for determining the robot's position on the field
  PhotonPoseEstimator photonPoseEstimator1;
  PhotonPoseEstimator photonPoseEstimator2;


  // AprilTag field layout for the 2024 Crescendo field
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Transformation from the robot to the camera
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); // Cam mounted facing forward, half a meter forward of center, half a meter up from center.


  boolean targetVisible1 = false;
  double targetYaw1 = 0.0;
  double targetPoseAmbiguity1 = 0.0;
  double range1 = 0.0;

  boolean targetVisible2 = false;
  double targetYaw2 = 0.0;
  double targetPoseAmbiguity2 = 0.0;
  double range2 = 0.0;

  double targetYaw = 0.0;
  double rangeToTarget = 0.0;

  /**
   * Constructs a new PhotonVision subsystem.
   */
  public Photonvision() {
    photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, robotToCam);
    photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2, robotToCam);
  }

  /**
   * This method is called periodically by the scheduler.
   * It updates the tracked targets and displays relevant information on the SmartDashboard.
   */
  @Override
  public void periodic() {
    var result1 = camera1.getLatestResult();
    var result2 = camera2.getLatestResult();

    // if (result1.hasTargets()){
    //   target1 = result1.getBestTarget();
    //   if (result1.getMultiTagResult().estimatedPose.isPresent) {
    //     Transform3d fieldToCamera = result1.getMultiTagResult().estimatedPose.best;
    //     SmartDashboard.putNumber("field to camera", fieldToCamera.getX());
    //   }
    // }

    // target2 = result2.hasTargets() ? result2.getBestTarget() : target2;



    if (result1.hasTargets()) {
        // Camera processed a new frame since last
        // Get the last one in the list.

            for (var tag : result1.getTargets()) {
                // IMPORTANT: CHANGE DA TAGRGET ID FOR STUFF AND THIGNS LOLOLOLOL
                // if (tag.getFiducialId() == 7) {
                if (true) {
                    // Found Tag 7, record its information

                    targetPoseAmbiguity1 = tag.getPoseAmbiguity();
                    targetYaw1 = tag.getYaw();
                    targetVisible1 = true;

                    range1 = PhotonUtils.calculateDistanceToTargetMeters(
                                        PhotonVisionConstants.CAMERA_ONE_HEIGHT,
                                        1.435, // From 2024 game manual for ID 7 | IMPORTANT TO CHANGE
                                        PhotonVisionConstants.CAMERA_ONE_ANGLE, // Rotation about Y = Pitch | UP IS POSITIVE
                                        Units.degreesToRadians(tag.getPitch()));

                }
            }
        }

        else {
            targetVisible1 = false;
        }
    if (result2.hasTargets()) {
        // Camera processed a new frame since last
        // Get the last one in the list.

            for (var tag : result2.getTargets()) {
                // IMPORTANT: CHANGE DA TAGRGET ID FOR STUFF AND THIGNS LOLOLOLOL
                // if (tag.getFiducialId() == 7) {
                if (true) {
                    // Found Tag 7, record its information

                    targetPoseAmbiguity2 = tag.getPoseAmbiguity();
                    targetYaw2 = tag.getYaw();
                    targetVisible2 = true;

                    range2 = PhotonUtils.calculateDistanceToTargetMeters(
                                        PhotonVisionConstants.CAMERA_TWO_HEIGHT,
                                        1.435, // From 2024 game manual for ID 7 | IMPORTANT TO CHANGE
                                        PhotonVisionConstants.CAMERA_TWO_ANGLE, // Rotation about Y = Pitch | UP IS POSITIVE
                                        Units.degreesToRadians(tag.getPitch()));
                }
            }
      }

      else {
        targetVisible2 = false;
      }

      if (targetPoseAmbiguity1 > targetPoseAmbiguity2)
      {
        targetYaw = targetYaw1;
        rangeToTarget = range1;
      }
      else
      {
        targetYaw = targetYaw2;
        rangeToTarget = range2;
      }

      SmartDashboard.putNumber("photon yaw", targetYaw);
      SmartDashboard.putNumber("range target", rangeToTarget);
  }

  /**
   * Gets the estimated global pose of the robot.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot.
   * @return An Optional containing the estimated robot pose, or empty if no pose could be estimated.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
    photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
    return targetPoseAmbiguity1 > targetPoseAmbiguity2 ? photonPoseEstimator2.update() : photonPoseEstimator1.update();
  }

  public double getYaw()
  {
    return targetYaw;
  }

  public double getRange()
  {
    return rangeToTarget;
  }
}
