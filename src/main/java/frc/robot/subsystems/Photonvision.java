package frc.robot.subsystems;

import static frc.robot.utils.GlobalsValues.SwerveGlobalValues.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues.PhotonVisionConstants;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** The PhotonVision subsystem handles vision processing using PhotonVision cameras. */
public class Photonvision extends SubsystemBase {
  // PhotonVision cameras
  PhotonCamera cameraleft = new PhotonCamera("Left");

  // Pose estimator for determining the robot's position on the field
  PhotonPoseEstimator photonPoseEstimatorleft;

  // AprilTag field layout for the 2024 Crescendo field
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  // Transformation from the robot to the camera
  // TODO: Make function to convert Translation2d to Translation3d
  Transform3d leftCameraPos =
      new Transform3d(
          conv2dTo3d(BACK_LEFT, PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER),
          new Rotation3d(0, 360 - PhotonVisionConstants.CAMERA_ONE_ANGLE_DEG, 150));

  PhotonTrackedTarget targetleft;
  boolean targetVisibleleft = false;
  double targetYawleft = -15.0;
  double targetPoseAmbiguityleft = 7157;
  double rangeleft = 0.0;


  double targetYaw = 0.0;
  double rangeToTarget = 0.0;

  PhotonPipelineResult resultleft;
  PhotonPipelineResult currentResult;

  boolean camleftTag = false;

  Optional<EstimatedRobotPose> currentPose;

  /** Constructs a new PhotonVision subsystem. */
  public Photonvision() {
    photonPoseEstimatorleft =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraleft,
            leftCameraPos);
  }
  /**
   * This method is called periodically by the scheduler. It updates the tracked targets and
   * displays relevant information on the SmartDashboard.
   */
  @Override
  public void periodic() {
    resultleft = cameraleft.getLatestResult();

    double distleft = 0;

    if (resultleft.hasTargets()) {
      targetleft = resultleft.getBestTarget();
      targetPoseAmbiguityleft = targetleft.getPoseAmbiguity();

      distleft = targetleft.getBestCameraToTarget().getTranslation().getNorm();
      // SmartDashboard.putNumber("distleft", distleft);

      // if (resultleft.getMultiTagResult().estimatedPose.isPresent) {
      //   Transform3d fieldToCamera = resultleft.getMultiTagResult().estimatedPose.best;
      //   SmartDashboard.putNumber("field to camera", fieldToCamera.getX());
      // }
    } else {
      targetPoseAmbiguityleft = 7157;
    }
    if(BasePIDGlobal.TEST_MODE == true) {
      SmartDashboard.putNumber("photon yaw", targetYaw);
      SmartDashboard.putNumber("range target", rangeToTarget);
      SmartDashboard.putNumber("april tag distance", getDistanceSubwoofer());
      SmartDashboard.putNumber("april tag yaw", getSubwooferYaw());
      SmartDashboard.putNumber("left cam ambiguity", targetPoseAmbiguityleft);
      SmartDashboard.putBoolean("left_targets", resultleft.hasTargets());
    }
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
    return photonPoseEstimatorleft.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    return photonPoseEstimatorleft.update();
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

  public double getOffset(PhotonCamera camera) {
    if (camera.getName().equals("Left")) {
      return PhotonVisionConstants.OFFSET_TOWARD_MID_LEFT;
    }

    return 0.0;
  }

  public double getPivotPosition() {
    // 10/14/2024 outside tuning
    // jayden why are you so bad at tuning
    // Desmos: https://www.desmos.com/calculator/naalukjxze
    double r = getDistanceSubwoofer();
    double f = -1.19541; // power 5
    double e = 18.0904; // power 4
    double d = -108.07; // power 3
    double c = 317.396; // power 2
    double b = -453.088; // power 1
    double a = 267.288; // constant

    // if (r == -1)
    // {
    //   return GlobalsValues.PivotGlobalValues.PIVOT_SUBWOOFER_ANGLE;
    // }
    return ((f * Math.pow(r, 5))
        + (e * Math.pow(r, 4))
        + (d * Math.pow(r, 3))
        + (c * Math.pow(r, 2))
        + (b * r)
        + a);
  }

  public Translation3d conv2dTo3d(Translation2d translation2d, double z) {
    return new Translation3d(translation2d.getX(), translation2d.getY(), z);
  }

  public double getYaw(PhotonCamera camera) {
    List<PhotonTrackedTarget> results = camera.getLatestResult().getTargets();
    boolean frontOfSubwoofer = false;

    for (PhotonTrackedTarget tag : results) {
      if (tag.getFiducialId() == 7 || tag.getFiducialId() == 4) {
        frontOfSubwoofer = true;
      }
    }

    SmartDashboard.putBoolean("front of subwoofer", frontOfSubwoofer);

    if (camera.getLatestResult().hasTargets() && frontOfSubwoofer) {
      return camera.getLatestResult().getBestTarget().getYaw();
    }
    return 4079;
  }

  public double getDistanceSubwoofer() {
    currentPose = getEstimatedGlobalPose();
    if (currentPose.isEmpty()) {
      return 687;
    }
    else {
      // 0.5, 5.5 coordinate for blue subwoofer
      // 16, 5.5 coordinate for red subwoofer
      if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
        return Math.sqrt(Math.pow(currentPose.get().estimatedPose.getTranslation().getX() - 16, 2) + Math.pow(currentPose.get().estimatedPose.getTranslation().getY() - 5.5, 2));
      } else {
        return Math.sqrt(Math.pow(currentPose.get().estimatedPose.getTranslation().getX() - 0.5, 2) + Math.pow(currentPose.get().estimatedPose.getTranslation().getY() - 5.5, 2));
      }
    }
  }

  public double getSubwooferYaw() {
    currentPose = getEstimatedGlobalPose();
    if (currentPose.isEmpty()) {
      return 8033;
    }
    else {
      // 0.5, 5.5 coordinate for blue subwoofer
      // 16, 5.5 coordinate for red subwoofer
      if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)) {
        return Math.toDegrees(Math.atan2(currentPose.get().estimatedPose.getTranslation().getY() - 5.5, currentPose.get().estimatedPose.getTranslation().getX() - 16));
      } else {
        return Math.toDegrees(Math.atan2(currentPose.get().estimatedPose.getTranslation().getY() - 5.5, currentPose.get().estimatedPose.getTranslation().getX() - 0.5));
      }
    }
  }
}