package frc.robot.subsystems;

import static frc.robot.utils.GlobalsValues.SwerveGlobalValues.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues.PhotonVisionConstants;
import java.util.Optional;

import org.opencv.photo.Photo;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** The PhotonVision subsystem handles vision processing using PhotonVision cameras. */
public class Photonvision extends SubsystemBase {
  // PhotonVision cameras
  PhotonCamera cameraleft = new PhotonCamera("Left");
  PhotonCamera cameraright = new PhotonCamera("Right");

  // Pose estimator for determining the robot's position on the field
  PhotonPoseEstimator photonPoseEstimatorleft;
  PhotonPoseEstimator photonPoseEstimatorright;

  // AprilTag field layout for the 2024 Crescendo field
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Transformation from the robot to the camera
  // TODO: Make function to convert Translation2d to Translation3d
  Transform3d leftCameraPos =
      new Transform3d(
          conv2dTo3d(BACK_LEFT, PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER),
          new Rotation3d(0, 360 - PhotonVisionConstants.CAMERA_ONE_ANGLE_DEG, 150));
  Transform3d rightCameraPos =
      new Transform3d(
          conv2dTo3d(BACK_RIGHT, PhotonVisionConstants.CAMERA_TWO_HEIGHT_METER),
          new Rotation3d(0, 360 - PhotonVisionConstants.CAMERA_TWO_ANGLE_DEG, 210));

  PhotonTrackedTarget targetleft;
  boolean targetVisibleleft = false;
  double targetYawleft = -15.0;
  double targetPoseAmbiguityleft = 0.0;
  double rangeleft = 0.0;

  PhotonTrackedTarget targetright;
  boolean targetVisibleright = false;
  double targetYawright = 15.0;
  double targetPoseAmbiguityright = 0.0;
  double rangeright = 0.0;

  double targetYaw = 0.0;
  double rangeToTarget = 0.0;

  PhotonPipelineResult resultleft;
  PhotonPipelineResult resultright;
  PhotonPipelineResult currentResult;

  boolean camleftTag = false;
  boolean camrightTag = false;

  /** Constructs a new PhotonVision subsystem. */
  public Photonvision() {
    photonPoseEstimatorleft =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraleft, leftCameraPos);
    photonPoseEstimatorright =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraright,
            rightCameraPos);
  }

  /**
   * This method is called periodically by the scheduler. It updates the tracked targets and
   * displays relevant information on the SmartDashboard.
   */
  @Override
  public void periodic() {
    resultleft = cameraleft.getLatestResult();
    resultright = cameraright.getLatestResult();

    double distleft = 0;
    double distright = 0;

    if (resultleft.hasTargets()){
      targetleft = resultleft.getBestTarget();
      targetPoseAmbiguityleft = targetleft.getPoseAmbiguity();
      SmartDashboard.putNumber("right cam ambiguity", targetPoseAmbiguityleft);


      distleft = targetleft.getBestCameraToTarget().getTranslation().getNorm();
      SmartDashboard.putNumber("distleft", distleft);

      // if (resultleft.getMultiTagResult().estimatedPose.isPresent) {
      //   Transform3d fieldToCamera = resultleft.getMultiTagResult().estimatedPose.best;
      //   SmartDashboard.putNumber("field to camera", fieldToCamera.getX());
      // }
    } 

    else {
      targetPoseAmbiguityleft = 1e9;
    }

    if (resultright.hasTargets()){
      targetright = resultright.getBestTarget();
      targetPoseAmbiguityright = targetright.getPoseAmbiguity();
      SmartDashboard.putNumber("left cam ambiguity", targetPoseAmbiguityright);

      // if (resultleft.getMultiTagResult().estimatedPose.isPresent) {
      //   Transform3d fieldToCamera = resultleft.getMultiTagResult().estimatedPose.best;
      //   SmartDashboard.putNumber("field to camera", fieldToCamera.getX());
      // }

      
      distright = targetright.getBestCameraToTarget().getTranslation().getNorm();
      SmartDashboard.putNumber("distright", distright);
    } 

    else{
      targetPoseAmbiguityright = 1e9;
    }

    SmartDashboard.putNumber("photon yaw", targetYaw);
    SmartDashboard.putNumber("range target", rangeToTarget);
    SmartDashboard.putNumber("april tag distance", getRange());

  }

  /**
   * Gets the estimated global pose of the robot.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot.
   * @return An Optional containing the estimated robot pose, or empty if no pose could be
   *     estimated.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimatorleft.setReferencePose(prevEstimatedRobotPose);
    photonPoseEstimatorright.setReferencePose(prevEstimatedRobotPose);
    return targetPoseAmbiguityleft > targetPoseAmbiguityright
        ? photonPoseEstimatorright.update()
        : photonPoseEstimatorleft.update();
  }

  /**
   * Horizontal
   *
   * <p>See <a
   * href="https://docs.photonvision.org/en/latest/docs/additional-resources/nt-api.html#getting-target-information">NetworkTables
   * API</a>
   */
  public double getYaw() {
    return targetYaw;
  }

  /**
   * Forward distance to target
   *
   * <p>See <a
   * href="https://docs.photonvision.org/en/latest/docs/additional-resources/nt-api.html#getting-target-information">NetworkTables
   * API</a>
   */
  public double getRange() {
    targetPoseAmbiguityleft = 0.0;
    targetPoseAmbiguityright = 0.0;
    if (resultleft.hasTargets()) {
      for (var tag : resultleft.getTargets()) {
        // TODO: Change the target ID depending on what we are looking for
        // if (tag.getFiducialId() == 7 || tag.getFiducialId() == 4) {
        if (true) {
          targetPoseAmbiguityleft = tag.getPoseAmbiguity();
          targetYawleft = tag.getYaw();
          targetVisibleleft = true;

          rangeleft =
              PhotonUtils.calculateDistanceToTargetMeters(
                  PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER,
                  1.435, // From right0right4 game manual for ID 7 | IMPORTANT TO CHANGE
                  Units.degreesToRadians(
                      PhotonVisionConstants
                          .CAMERA_ONE_ANGLE_DEG), // Rotation about Y = Pitch | UP IS POSITIVE
                  Units.degreesToRadians(tag.getPitch()));
        }
      }
    } else {
      targetVisibleleft = false;
    }
    if (resultright.hasTargets()) {
      for (var tag : resultright.getTargets()) {
        // TODO: Change the target ID depending on what we are looking for
        // if (tag.getFiducialId() == 7 || tag.getFiducialId() == 4) {
        if (true) {
          targetPoseAmbiguityright = tag.getPoseAmbiguity();
          targetYawright = tag.getYaw();
          targetVisibleright = true;

          rangeright =
              PhotonUtils.calculateDistanceToTargetMeters(
                  PhotonVisionConstants.CAMERA_TWO_HEIGHT_METER,
                  1.435, // From right0right4 game manual for ID 7 | IMPORTANT TO CHANGE
                  Units.degreesToRadians(
                      PhotonVisionConstants
                          .CAMERA_TWO_ANGLE_DEG), // Rotation about Y = Pitch | UP IS POSITIVE
                  Units.degreesToRadians(tag.getPitch()));
        }
      }
    } else {
      targetVisibleright = false;
    }
    if (targetPoseAmbiguityleft > targetPoseAmbiguityright && targetVisibleleft) {
      return rangeleft;
    } else if (targetVisibleright) {
      return rangeright;
    } else {
      return 0.0;
    }
  }

  public double getOffset(PhotonCamera camera) {
    if (camera.getName() == "left")
    {
      return PhotonVisionConstants.OFFSET_TOWARD_MID_LEFT;
    }

    if (camera.getName() == "right")
    {
      return PhotonVisionConstants.OFFSET_TOWARD_MID_RIGHT;
    }

    return 0.0;
  }

  public double getPivotPosition() {
    return (-0.273166 * Math.pow(getRange(), 5)
        + 4.16168 * Math.pow(getRange(), 4)
        + -23.6466 * Math.pow(getRange(), 3)
        + 60.022 * Math.pow(getRange(), 2)
        + getRange() * -58.4714
        + 27.1329);
  }

  public Translation3d conv2dTo3d(Translation2d translation2d, double z) {
    return new Translation3d(translation2d.getX(), translation2d.getY(), z);
  }

  public PhotonCamera getBestCamera() {
    return (targetPoseAmbiguityleft > targetPoseAmbiguityright) ? cameraright : cameraleft;
  }
  
  public double getYaw(PhotonCamera camera) {
    if (camera.getLatestResult().hasTargets())
    {
      return camera.getLatestResult().getBestTarget().getYaw();
    }
    return -1;
  }
}
