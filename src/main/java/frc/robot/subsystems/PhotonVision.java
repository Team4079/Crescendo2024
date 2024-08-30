// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera camera1 = new PhotonCamera("Camera One");
  PhotonCamera camera2 = new PhotonCamera("Camera Two");
  PhotonTrackedTarget target1;
  PhotonTrackedTarget target2;
  PhotonPoseEstimator photonPoseEstimator;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  public PhotonVision() {
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, robotToCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var result1 = camera1.getLatestResult();
    var result2 = camera2.getLatestResult();


    if (result1.hasTargets()){
      target1 = result1.getBestTarget();
      if (result1.getMultiTagResult().estimatedPose.isPresent) {
        Transform3d fieldToCamera = result1.getMultiTagResult().estimatedPose.best;
        SmartDashboard.putNumber("field to camera", fieldToCamera.getX());
      }
    }

    if (result2.hasTargets()){
      target2 = result2.getBestTarget();
    }
  }

public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
}
