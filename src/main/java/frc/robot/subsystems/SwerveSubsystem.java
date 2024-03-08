package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

// import edu.wpi.first.math.controller.PIDController;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.MotorConstants;
import frc.robot.utils.Constants.SwerveConstants;

/**
 * The {@link SwerveSubsystem} class includes all the motors to drive the robot.
 * 
 *
 * 
 */
public class SwerveSubsystem extends SubsystemBase {
  private SwerveModule[] modules;
  private Rotation2d gyroAngle;
  private Pigeon2 pidggy;
  private final SwerveDriveKinematics sKinematics;

  public SwerveDriveOdometry swerveOdometry;
  public Pose2d swerveOdomeryPose2d;

  private double rot;
  private double turnSpeed = 0;

  /** Creates a new DriveTrain. */
  public SwerveSubsystem() {
    sKinematics = Constants.SwerveConstants.kinematics;
    gyroAngle = Rotation2d.fromDegrees(0);
    pidggy = new Pigeon2(16);
    pidggy.reset();

    modules = new SwerveModule[] {
        new SwerveModule(
            MotorConstants.FRONT_LEFT_DRIVE_ID,
            MotorConstants.FRONT_LEFT_STEER_ID,
            MotorConstants.FRONT_LEFT_CAN_CODER_ID,
            SwerveConstants.CANCoderValue9),
        new SwerveModule(
            MotorConstants.FRONT_RIGHT_DRIVE_ID,
            MotorConstants.FRONT_RIGHT_STEER_ID,
            MotorConstants.FRONT_RIGHT_CAN_CODER_ID,
            SwerveConstants.CANCoderValue10),
        new SwerveModule(
            MotorConstants.BACK_LEFT_DRIVE_ID,
            MotorConstants.BACK_LEFT_STEER_ID,
            MotorConstants.BACK_LEFT_CAN_CODER_ID,
            SwerveConstants.CANCoderValue11),
        new SwerveModule(
            MotorConstants.BACK_RIGHT_DRIVE_ID,
            MotorConstants.BACK_RIGHT_STEER_ID,
            MotorConstants.BACK_RIGHT_CAN_CODER_ID,
            SwerveConstants.CANCoderValue12)
    };

    swerveOdometry = new SwerveDriveOdometry(sKinematics, gyroAngle, getModulePositions());
    swerveOdomeryPose2d = new Pose2d();

    addRotorPositionsforModules();

    // Makes the rotation smooth (in a circle)
    SwerveConstants.BasePIDConstants.pathTranslationPID.enableContinuousInput(-Math.PI, Math.PI);

    /**
     * PathPlanner Direction Values
     * Forward +x
     * Backward -x
     * Left -y
     * Right +y
     */
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::customPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getAutoSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::chassisSpeedsDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        SwerveConstants.BasePIDConstants.pathFollower,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  /**
   * Drives the robot using the joystick input
   * 
   * @param forwardSpeed double Speed value in meters per second
   * @param leftSpeed double Speed value in meters per second
   * @param joyStickInput double joystick input value
   * @param isFieldOriented boolean value to determine if the robot is field oriented
   * @return void
   */
  public void drive(double forwardSpeed, double leftSpeed, double joyStickInput, boolean isFieldOriented) {
    ChassisSpeeds speeds;

    turnSpeed = joyStickInput * MotorConstants.TURN_CONSTANT;

    // Runs robot/field-oriented based on the boolean value of isFieldOriented
    if (isFieldOriented) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          forwardSpeed,
          leftSpeed,
          turnSpeed,
          getRotationPidggy());
    } else {
      speeds = new ChassisSpeeds(
          forwardSpeed,
          leftSpeed,
          joyStickInput);
    }

    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);

      SmartDashboard.putNumber("MotorRot" + i, modules[i].getRotationDegree() % 360);
    }
  }

  /**
   * Gets the position of the modules in the swerve drive train
   * @param void
   * @return SwerveModulePosition[] array of the position of the modules
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /**
   * Gets the rotation of the pigeon
   * @param void
   * @return Rotation2d rotation of the pigeon in degrees
   */
  public Rotation2d getRotationPidggy() {
    rot = -pidggy.getRotation2d().getDegrees();
    return Rotation2d.fromDegrees(rot);
  }

  /**
   * Resets the pigeon to zero
   * @param void
   * @return None
   */
  public void zeroHeading() {
    pidggy.reset();
  }

  /**
   * Resets the drive encoders
   * @param void
   * @return None
   */
  public void resetDriveEncoders() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].resetEncoders();
    }
  }

  /**
   * Gets the yaw of the pigeon in double degrees
   * @param void
   * @return double yaw of the pigeon in degrees
   */
  public double getYaw() {
    return pidggy.getYaw().getValue();
  }

  /**
   * Gets the heading of the pigeon in double degrees
   * @param void
   * @return double heading of the pigeon in degrees
   */
  public double pgetHeading() {
    return (pidggy.getYaw().getValue() % 360);
  }

  // Speed modifiers
  /**
   * Configures the slow mode of the robot
   * @param void
   * @return None
   */
  public void configSlowMode() {
    MotorConstants.SLOW_MODE = !MotorConstants.SLOW_MODE;
  }

  /**
   * Gets the slow mode of the robot in boolean value
   * @param void
   * @return boolean value of the slow mode
   */
  public boolean getSlowMode() {
    return MotorConstants.SLOW_MODE;
  }

  /**
   * Configures the acorn mode of the robot
   * @param void
   * @return None
   */
  public void configAAcornMode() {
    MotorConstants.AACORN_MODE = !MotorConstants.AACORN_MODE;
  }

  /**
   * Gets the acorn mode of the robot in boolean value
   * @param void
   * @return boolean value of the acorn mode
   */
  public boolean getAAcornMode() {
    return MotorConstants.AACORN_MODE;
  }

  // Position methods - used for odometry
  /**
   * Gets the pose of the robot in Pose2d
   * @param void
   * @return Pose2d pose of the robot
   */
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Resets the heading of the robot to 0
   * @param void
   * @return None
   */
  public void newPose() {
    swerveOdometry.resetPosition(Rotation2d.fromDegrees(pgetHeading()), getModulePositions(),
        new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  /**
   * Resets the pose of the robot to a custom pose
   * @param poses Pose2d pose of the robot
   * @return None
   */
  public void customPose(Pose2d poses) {
    swerveOdometry.resetPosition(Rotation2d.fromDegrees(pgetHeading()), getModulePositions(), poses);
  }

  /**
   * Updates odometry and gets heading of the pigeon
   * @param void
   * @return None
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidggy.getYaw().refresh();
    Rotation2d headingGyroAnglething = Rotation2d.fromDegrees(pgetHeading());
    swerveOdomeryPose2d = swerveOdometry.update(headingGyroAnglething, getModulePositions());

    SmartDashboard.putNumber("heading", pgetHeading());
    gyroAngle = getRotationPidggy();
  }

  /**
   * Adds rotor positions for the modules
   * @param void
   * @return None
   */
  public void addRotorPositionsforModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setRotorPos();
    }
  }

  /**
   * Resets odometry to a custom pose
   * @param pose Pose2d
   * @return None
   */
  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getModulePositions(), pose);
  }

  /**
   * Passes in the module states to the modules (speed and rotation)
   * @param states SwerveModuleState[] array of the module states
   * @return None
   */
  public void outputModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  /**
   * Gets the auto speeds of the robot in meters per second
   * @param void
   * @return ChassisSpeeds auto speeds of the robot
   */
  public ChassisSpeeds getAutoSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0));
  }

  /**
   * Drives the robot using the chassis speeds in meters per second
   * @param chassisSpeeds chassis speeds
   * @return None
   */
  public void chassisSpeedsDrive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  /**
   * Stops the modules
   * @param void
   * @return None
   */
  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /**
   * Stops the robot
   * @param void
   * @return None
   */
  public void stop() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].stop();
    }
  }
}