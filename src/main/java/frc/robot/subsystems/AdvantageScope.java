// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import java.lang.System.Logger;
// import java.lang.reflect.Field;

// import com.ctre.phoenix6.hardware.Pigeon2;

// import edu.wpi.first.math.estimator.PoseEstimator;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class AdvantageScope extends SubsystemBase {
//   private SwerveSubsystem swerveSubsystem;
//   private SwerveModule swerveModule;
//   private Pigeon2 pidggy;
//   private Pose2d m_previousPose;
//   private SwerveDrivePoseEstimator m_poseEstimator;
//   private Field2d m_field;
//   private Rotation2d m_currentHeading;

//   /** Creates a new AdvantageScope. */
//   public AdvantageScope(SwerveSubsystem swerveSubsystem, Pigeon2 pidggy, SwerveModule swerveModule) {
//     this.swerveSubsystem = swerveSubsystem;
//     this.swerveModule = swerveModule;
//     this.pidggy = pidggy;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//     /**
//    * Update robot pose
//    */
//   private void updatePose() {
//     // Save previous pose
//     m_previousPose = swerveSubsystem.getPose();

//     // Update pose based on odometry
//     m_poseEstimator.update(pidggy.getRotation2d(), swerveModule.getPosition());

//     // Update current heading
//     m_currentHeading = new Rotation2d(swerveSubsystem.getPose().getX() - m_previousPose.getX(), swerveSubsystem.getPose().getY() - m_previousPose.getY());

//     // Get estimated poses from VisionSubsystem
//     var visionEstimatedRobotPoses = swerveSubsystem.getInstance().getEstimatedGlobalPoses();

//     // Exit if no valid vision pose estimates
//     if (visionEstimatedRobotPoses.isEmpty()) return;

//     // Add vision measurements to pose estimator
//     for (var visionEstimatedRobotPose : visionEstimatedRobotPoses) {
//       if (visionEstimatedRobotPose.estimatedPose.toPose2d().getTranslation().getDistance(m_previousPose.getTranslation()) > 1.0) continue;
//       m_poseEstimator.addVisionMeasurement(visionEstimatedRobotPose.estimatedPose.toPose2d(), visionEstimatedRobotPose.timestampSeconds);
//     }
//   }

//   /**
//    * Log DriveSubsystem outputs
//    */
//   private void logOutputs() {
//     Logger.recordOutput(getName() + POSE_LOG_ENTRY, getPose());
//     Logger.recordOutput(getName() + ACTUAL_SWERVE_STATE_LOG_ENTRY, getModuleStates());
//   }

//   /**
//    * SmartDashboard indicators
//    */
//   private void smartDashboard() {
//     m_field.setRobotPose(swerveSubsystem.getPose());
//   }
// }
