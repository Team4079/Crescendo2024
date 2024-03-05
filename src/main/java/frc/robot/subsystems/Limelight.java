// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

/**
 * The {@link Limelight} class includes all the methods to interact with the
 * limelight. (Limelety)
 * 
 * <p> The limelight gets values based on the pipeline and mode set.
 * 
 */
public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final NetworkTable m_limelightTable;
  LimelightHelpers.LimelightResults llresults;
  double tv, tx, ty, ta, ts = 0.0;
  private Pose2d robotPose_FieldSpace;

  private double[] robotPoseTargetSpace;

  private Field2d field = new Field2d();

  public Limelight() {
    m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    // HttpCamera limelightCamera = new HttpCamera("limelight",
    // "http://limelight.local:5801/stream.mjpg");
    llresults = LimelightHelpers.getLatestResults("limelight");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tv = m_limelightTable.getEntry("tv").getDouble(0);
    tx = m_limelightTable.getEntry("tx").getDouble(0);
    ty = m_limelightTable.getEntry("ty").getDouble(0);
    ta = m_limelightTable.getEntry("ta").getDouble(0);
    robotPoseTargetSpace = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_targetspace")
        .getDoubleArray(new double[6]);

    if (DriverStation.getAlliance().equals(DriverStation.Alliance.Red)) {
      robotPose_FieldSpace = llresults.targetingResults.getBotPose2d_wpiRed();
    } else {
      robotPose_FieldSpace = llresults.targetingResults.getBotPose2d_wpiBlue();
    }

    // robotPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
    SmartDashboard.putNumber("April Tag X", LimelightHelpers.getTX("limelight"));
    field.setRobotPose(robotPose_FieldSpace);
    SmartDashboard.putData("Field Vision", field);
    for (int i = 0; i < robotPoseTargetSpace.length; i++) {
      SmartDashboard.putNumber("robotPoseTargetSpace" + i, robotPoseTargetSpace[i]);
    }
  }

  /**
   * Returns the x value of the april tag from the limelight
   * 
   * @return tx value
   */
  public double getTx() {
    return tx;
  }

  /**
   * Returns the area value from the limelight
   * 
   * @return ta value
   */
  public double getTa() {
    return ta;
  }

  /**
   * Returns the fullness value from the limelight
   * 
   * @return tv value
   */
  public double getTv() {
    return tv;
  }

  /**
   * Returns the y value from the limelight
   * @return ty value
   */
  public double getTy() {
    return ty;
  }

  // index from 0
  // 0 is left-right distance from tag (left is +, right is -, accurate to +- 5cm
  // per meter)
  // 1 is undocumented
  // 2 is forward-backward distance from tag (forward is +, backward is -,
  // accurate to +- 5cm per meter)
  // 3 is undocumented
  // 4 is rotation (clockwise is -) (accurate to +-0.5 a degree)
  // 5

  /**
   * Returns the robot pose in target space
   * 
   * @return Robot Pose in target space
   */
  public double[] getRobotPose_TargetSpace2D() {
    return robotPoseTargetSpace;
  }

  public boolean isTarget() {
    return LimelightHelpers.getTX("limelight") != 0;
  }

  /**
   * Sets the pipeline of the limelight
   * 
   * @param pipeline The pipeline to set the limelight
   */
  public void setPipeline(double pipeline) {
    m_limelightTable.getEntry("pipeline").setNumber(pipeline);
  }

  /**
   * Sets the mode of the limelight (fiducial, retroflective)
   * 
   * @param mode The mode to set the limelight
   */
  public void setAdvanced(double mode) {
    m_limelightTable.getEntry("advanced_mode").setNumber(mode);
  }

  /**
   * Gets the latency of the limelight
   * 
   * @return Latency in ms
   */
  public double getLatency() {
    return llresults.targetingResults.latency_capture;
  }

  /**
   * Returns the tag number of the april tag
   * 
   * @return April Tag number
   */
  public double getTag() {
    return LimelightHelpers.getFiducialID("");
  }

  /**
   * Returns the robot position in the field
   * 
   * @return Robot Position
   */
  public Pose2d getRobotPosition() {
    return robotPose_FieldSpace;
  }

}