// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Photonvision extends SubsystemBase {
  /** Creates a new Photonvision. */

  public Field2d field;


  PhotonCamera camera1 = new PhotonCamera("Camera One");
  PhotonCamera camera2 = new PhotonCamera("Camera Two");
  PhotonTrackedTarget target1;
  PhotonTrackedTarget target2;
  PhotonPoseEstimator photonPoseEstimator1;
  PhotonPoseEstimator photonPoseEstimator2;

  double ambiguity1;
  double ambiguity2;

  PhotonPipelineResult results1;
  PhotonPipelineResult results2;

  double range1;
  double range2;

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  Transform3d robotToCam1 = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  Transform3d robotToCam2 = new Transform3d(new Translation3d(-0.5, 0.0, -0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

  public Photonvision() {
    photonPoseEstimator1 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera1, robotToCam1);
    photonPoseEstimator2 = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera2, robotToCam2);
  }

  @Override
  public void periodic() {
    // results1 = camera1.getLatestResult();
    // results2 = camera2.getLatestResult();

    

    // SmartDashboard.putData("AACORN", field);
    // This method will be called once per scheduler run
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose1(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator1.update();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose2(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator2.update();
  }

  public Optional<EstimatedRobotPose> getBestEstimatedPose(Pose2d prevEstimatedRobotPose) {
    if (results1.hasTargets() && results2.hasTargets()) {
      if (results1.getTargets().get(0).getPoseAmbiguity() < results2.getTargets().get(0).getPoseAmbiguity()) {
        photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator1.update();
      }
      else {
        photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator2.update();
      }
    }
    else if (results1.hasTargets()) {
      photonPoseEstimator1.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator1.update();
    }
    else if (results2.hasTargets()) {
      photonPoseEstimator2.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator2.update();
    }
    else {
      return Optional.empty();
    }
  }

  public boolean hasTargets1() {
    return results1.hasTargets();
  }

  public boolean hasTargets2() {
    return results2.hasTargets();
  }

  public void setAmbiguity() {

    Pose2d newPose;

    if (results1.hasTargets()) {
      try {
        ambiguity1 = camera1.getLatestResult().getBestTarget().getPoseAmbiguity();
      }
      catch(NullPointerException e) {
        System.out.println("Jayden had a skill issue");
      }
    } 
    else {
      ambiguity1 = 1;
    }

    if (results2.hasTargets()) {
      try {
        ambiguity2 = camera2.getLatestResult().getBestTarget().getPoseAmbiguity();
      }
      catch(NullPointerException e) {
        System.out.println("Jayden had a huge skill issue");
      }
    } 
    else {
      ambiguity2 = 1;
    }

    // field.setRobotPose(swerveEstimator.getEstimatedPosition());
  }

  public double getDistancePhoton()  {
    return ambiguity1 < ambiguity2 ? range1 : range2;
  }

  public double getTimestampSeconds() {
    return results1.getTimestampSeconds();
  }
}
