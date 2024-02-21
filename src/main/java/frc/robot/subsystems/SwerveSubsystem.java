package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

// import edu.wpi.first.math.controller.PIDController;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Robot;
import frc.robot.utils.Hell;
import frc.robot.utils.Hell.MotorConstants;
import frc.robot.utils.Hell.SwerveConstants;

@SuppressWarnings("unused") // Used in order to remove warnings
public class SwerveSubsystem extends SubsystemBase {
  // private SwerveModule frontLeft;
  // private SwerveModule frontRight;
  // private SwerveModule backLeft;
  // private SwerveModule backRight;
  private SwerveModule[] modules;
  private SwerveDrivePoseEstimator estimator;
  private Rotation2d gyroAngle;
  private Pigeon2 pidggy;
  private final SwerveDriveKinematics sKinematics;


  public SwerveDriveOdometry swerveOdometry;
  public Pose2d swerveOdomeryPose2d;

  private double rot;
  private double turnSpeed = 0;

  private int slow = 0;

  private Field2d field = new Field2d();

  /** Creates a new DriveTrain. */
  public SwerveSubsystem() {
    sKinematics = Hell.SwerveConstants.kinematics;
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

    estimator = new SwerveDrivePoseEstimator(
        SwerveConstants.kinematics,
        getRotationPidggy(),
        getModulePositions(),
        SwerveConstants.STARTING_POSE);

    addRotorPositionsforModules();

    // forward + x
    // backward - x
    // left - y
    // right + y
    // theta = we dont know lol


    SwerveConstants.BasePIDConstants.pathTranslationPID.enableContinuousInput(-Math.PI, Math.PI);

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::customPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getAutoSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::chassisSpeedsDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        SwerveConstants.BasePIDConstants.pathFollwer,
        () -> false,
        this // Reference to this subsystem to set requirements
    );

    // PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
  }

  public void drive(double forwardSpeed, double leftSpeed, double joyStickInput, boolean isFieldOriented) {
    ChassisSpeeds speeds;

    turnSpeed = joyStickInput * MotorConstants.TURN_CONSTANT;

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

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  public Rotation2d getRotationPidggy() {
    rot = -pidggy.getRotation2d().getDegrees();
    return Rotation2d.fromDegrees(rot);
  }

  public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d newPose) {
    estimator.resetPosition(gyroAngle, modulePositions, newPose);
  }

  public void zeroHeading() {
    pidggy.reset();
  }

  public void resetDriveEncoders() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].resetEncoders();
    }
  }

  public double getYaw() {
    return pidggy.getYaw().getValue();
  }

  public double pgetHeading() {
    return (pidggy.getYaw().getValue() % 360);
  }

  public void configSlowMode() {
    MotorConstants.SLOW_MODE = !Hell.MotorConstants.SLOW_MODE;
  }

  public boolean getSlowMode() {
    return MotorConstants.SLOW_MODE;
  }

  public void configAAcornMode() {
    MotorConstants.AACORN_MODE = !Hell.MotorConstants.AACORN_MODE;
  }

  public boolean getAAcornMode() {
    return MotorConstants.AACORN_MODE;
  }

  public void updatePosition(Limelight limelety){
    this.addVision(limelety.getRobotPosition());
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void newPose() {
    swerveOdometry.resetPosition(Rotation2d.fromDegrees(pgetHeading()), getModulePositions(), new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
  }

  public void customPose(Pose2d poses) {
    swerveOdometry.resetPosition(Rotation2d.fromDegrees(pgetHeading()), getModulePositions(), poses);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidggy.getYaw().refresh();
    Rotation2d headingGyroAnglething = Rotation2d.fromDegrees(pgetHeading());
    swerveOdomeryPose2d = swerveOdometry.update(headingGyroAnglething, getModulePositions());

    // field.setRobotPose(getPose());

    gyroAngle = getRotationPidggy();
    // SmartDashboard.putNumber("Gyro Angle", gyroAngle.getDegrees());
    estimator.update(gyroAngle, getModulePositions());
  }

  public void stopModules() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  public void test(int moduleNum, double driveSpeed, double rotationSpeed) {
    SwerveModule module = modules[moduleNum];

    module.setDriveSpeed(driveSpeed);
    module.setSteerSpeed(rotationSpeed);
  }

  public void addRotorPositionsforModules() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].zeroRotorPosition();
    }
  }

  public void stop() {
    for (int i = 0; i < modules.length; i++) {
      modules[i].stop();
    }
  }

  // public void updateEstimator() {
  //   estimator.update(getRotationPidggy(), getModulePositions());
  // }

  // public void addVision() {
  //   estimator.addVisionMeasurement(null, Timer.getFPGATimestamp());
  // }


  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(Rotation2d.fromDegrees(getYaw()), getModulePositions(), pose);
  }

  public void addVision(Pose2d visionPose) {
    estimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
  }

  public void outputModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  // public double getX() {
  //   return estimator.getEstimatedPosition().getTranslation().getX();
  // }

  // public double getY() {
  //   return estimator.getEstimatedPosition().getTranslation().getY();
  // }

  public ChassisSpeeds getAutoSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(0));
  }

  public void chassisSpeedsDrive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        states, MotorConstants.MAX_SPEED);

    for (int i = 0; i < modules.length; i++) {
      modules[i].setState(states[i]);
    }
  }
}