package frc.robot.subsystems;

import static frc.robot.utils.GlobalsValues.SwerveGlobalValues.BASE_LENGTH_ERICK_TRAN;

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
import java.util.Optional;
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
  PhotonCamera camera1 = new PhotonCamera("Left");
  PhotonCamera camera2 = new PhotonCamera("Right");

  // Tracked targets from the cameras
  PhotonTrackedTarget target1;
  PhotonTrackedTarget target2;

  // Pose estimator for determining the robot's position on the field
  PhotonPoseEstimator photonPoseEstimator1;
  PhotonPoseEstimator photonPoseEstimator2;

  // AprilTag field layout for the 2024 Crescendo field
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Transformation from the robot to the camera
  // TODO: Make function to convert Translation2d to Translation3d
  Transform3d leftCameraPos =
      new Transform3d(
          new Translation3d(
              -BASE_LENGTH_ERICK_TRAN / 2,
              -BASE_LENGTH_ERICK_TRAN / 2,
              PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER),
          new Rotation3d(0, 360 - PhotonVisionConstants.CAMERA_ONE_ANGLE_DEG, 150));
  Transform3d rightCameraPos =
      new Transform3d(
          new Translation3d(
              BASE_LENGTH_ERICK_TRAN / 2,
              -BASE_LENGTH_ERICK_TRAN / 2,
              PhotonVisionConstants.CAMERA_TWO_HEIGHT_METER),
          new Rotation3d(0, 360 - PhotonVisionConstants.CAMERA_TWO_ANGLE_DEG, 210));

  boolean targetVisible1 = false;
  double targetYaw1 = -15.0;
  double targetPoseAmbiguity1 = 0.0;
  double range1 = 0.0;

  boolean targetVisible2 = false;
  double targetYaw2 = 15.0;
  double targetPoseAmbiguity2 = 0.0;
  double range2 = 0.0;

  double targetYaw = 0.0;
  double rangeToTarget = 0.0;

  PhotonPipelineResult result1;
  PhotonPipelineResult result2;

  /** Constructs a new PhotonVision subsystem. */
  public Photonvision() {
    photonPoseEstimator1 =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, leftCameraPos);
    photonPoseEstimator2 =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera2,
            rightCameraPos);
  }

  /**
   * This method is called periodically by the scheduler. It updates the tracked targets and
   * displays relevant information on the SmartDashboard.
   */
  @Override
  public void periodic() {
    result1 = camera1.getLatestResult();
    result2 = camera2.getLatestResult();

    // if (result1.hasTargets()){
    //   target1 = result1.getBestTarget();
    //   if (result1.getMultiTagResult().estimatedPose.isPresent) {
    //     Transform3d fieldToCamera = result1.getMultiTagResult().estimatedPose.best;
    //     SmartDashboard.putNumber("field to camera", fieldToCamera.getX());
    //   }
    // }

    // target2 = result2.hasTargets() ? result2.getBestTarget() : target2;

    if (targetPoseAmbiguity1 > targetPoseAmbiguity2) {
      targetYaw = targetYaw1;
      rangeToTarget = range1;
    } else {
      targetYaw = targetYaw2;
      rangeToTarget = range2;
    }

    SmartDashboard.putNumber("photon yaw", targetYaw);
    SmartDashboard.putNumber("range target", rangeToTarget);
    SmartDashboard.putNumber("april tag distance", getDistanceSubwoofer());
  }

  /**
   * Gets the estimated global pose of the robot.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot.
   * @return An Optional containing the estimated robot pose, or empty if no pose could be
   *     estimated.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
    photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
    return targetPoseAmbiguity1 > targetPoseAmbiguity2
        ? photonPoseEstimator2.update()
        : photonPoseEstimator1.update();
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
    return rangeToTarget;
  }

  public double getDistanceSubwoofer() {
    targetPoseAmbiguity1 = 0.0;
    targetPoseAmbiguity2 = 0.0;
    if (result1.hasTargets()) {
      // Camera processed a new frame since last
      // Get the last one in the list.
      for (var tag : result1.getTargets()) {
        // TODO: CHANGE DA TAGRGET ID FOR STUFF AND THIGNS LOLOLOLOL
        // if (tag.getFiducialId() == 7 || tag.getFiducialId() == 4) {
        if (true) {
          // Found Tag 7, record its information

          targetPoseAmbiguity1 = tag.getPoseAmbiguity();
          targetYaw1 = tag.getYaw();
          targetVisible1 = true;

          range1 =
              PhotonUtils.calculateDistanceToTargetMeters(
                  PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER,
                  1.435, // From 2024 game manual for ID 7 | IMPORTANT TO CHANGE
                  Units.degreesToRadians(
                      PhotonVisionConstants
                          .CAMERA_ONE_ANGLE_DEG), // Rotation about Y = Pitch | UP IS POSITIVE
                  Units.degreesToRadians(tag.getPitch()));
        }
      }
    } else {
      targetVisible1 = false;
    }
    if (result2.hasTargets()) {
      // Camera processed a new frame since last
      // Get the last one in the list.

      for (var tag : result2.getTargets()) {
        // TODO: CHANGE DA TAGRGET ID FOR STUFF AND THIGNS LOLOLOLOL
        // if (tag.getFiducialId() == 7 || tag.getFiducialId() == 4) {
        if (true) {
          // Found Tag 7, record its information

          targetPoseAmbiguity2 = tag.getPoseAmbiguity();
          targetYaw2 = tag.getYaw();
          targetVisible2 = true;

          range2 =
              PhotonUtils.calculateDistanceToTargetMeters(
                  PhotonVisionConstants.CAMERA_TWO_HEIGHT_METER,
                  1.435, // From 2024 game manual for ID 7 | IMPORTANT TO CHANGE
                  Units.degreesToRadians(
                      PhotonVisionConstants
                          .CAMERA_TWO_ANGLE_DEG), // Rotation about Y = Pitch | UP IS POSITIVE
                  Units.degreesToRadians(tag.getPitch()));
        }
      }
    } else {
      targetVisible2 = false;
    }
    if (targetPoseAmbiguity1 > targetPoseAmbiguity2 && targetVisible1) {
      return range1;
    } else if (targetVisible2) {
      return range2;
    } else {
      return 0.0;
    }
  }

  public double getOffset() {
    if (targetPoseAmbiguity1 > targetPoseAmbiguity2 && targetVisible1) {
      return PhotonVisionConstants.OFFSET_TOWARD_MID_LEFT;
    } else if (targetVisible2) {
      return PhotonVisionConstants.OFFSET_TOWARD_MID_RIGHT;
    } else {
      return 0.0;
    }
  }

  public double getPivotPosition() {
    // return (-0.288051 * Math.pow(getDistance(), 5) + 4.37563 * Math.pow(getDistance(), 4) +
    // -24.8164 * Math.pow(getDistance(), 3) + 63.047 * Math.pow(getDistance(), 2) + getDistance() *
    // -61.9595 + 28.877);
    // return (-0.288051 * Math.pow(getDis(), 5) + 4.37563 * Math.pow(getDis(), 4) + -24.8164 *
    // Math.pow(getDis(), 3) + 63.047 * Math.pow(getDis(), 2) + getDis() * -61.9595 + 28.577);
    return (-0.273166 * Math.pow(getDistanceSubwoofer(), 5)
        + 4.16168 * Math.pow(getDistanceSubwoofer(), 4)
        + -23.6466 * Math.pow(getDistanceSubwoofer(), 3)
        + 60.022 * Math.pow(getDistanceSubwoofer(), 2)
        + getDistanceSubwoofer() * -58.4714
        + 27.1329); // ( 27.0538)
    // return (-1
  }
}
