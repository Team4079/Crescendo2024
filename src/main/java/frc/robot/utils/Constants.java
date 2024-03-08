// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  private static final String UTILITY_CLASS = "Utility class";

  private Constants() {
    throw new IllegalStateException(UTILITY_CLASS);
  }

  public static class MotorConstants {
    private MotorConstants() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    // Motor CAN ID Values
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_RIGHT_STEER_ID = 3;
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int BACK_LEFT_STEER_ID = 5;
    public static final int BACK_LEFT_DRIVE_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 7;
    public static final int BACK_RIGHT_DRIVE_ID = 8;
    public static final int FRONT_LEFT_CAN_CODER_ID = 9;
    public static final int FRONT_RIGHT_CAN_CODER_ID = 10;
    public static final int BACK_LEFT_CAN_CODER_ID = 11;
    public static final int BACK_RIGHT_CAN_CODER_ID = 12;

    // Motor Property Values
    public static final double MAX_SPEED = 5.76;
    public static final double MAX_ANGULAR_SPEED = (14 * Math.PI) / 3;
    public static final double ENCODER_COUNTS_PER_ROTATION = 1; // 2048 for v5, 1 for v6 (rotations)
    public static final double STEER_MOTOR_GEAR_RATIO = 150.0 / 7; // 24
    public static final double DRIVE_MOTOR_GEAR_RATIO = 5.9;
    public static final double WHEEL_DIAMETER = 0.1;
    public static final double SPEED_CONSTANT = 0.6; // 0.4
    public static final double AACORN_SPEED = 0.95;
    public static final double SLOW_SPEED = 0.3;
    public static final double TURN_CONSTANT = 0.3; // 0.3
    public static double HEADING = 0.0;

    // Motor Speed Manipulation Values
    public static boolean SLOW_MODE = false;
    public static boolean AACORN_MODE = true;
  }

  public static class SwerveConstants {
    private SwerveConstants() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    public static final double robotSize = 0.43105229381; // Keep constant *ideally*

    // Motor Locations (Relative to the center in meters)
    public static final Translation2d frontLeftLocation = new Translation2d(0.3048, -0.3048); // (0.263525, -0.263525);
    public static final Translation2d frontRightLocation = new Translation2d(0.3048, 0.3048); // (0.263525, 0.263525);
    public static final Translation2d backLeftLocation = new Translation2d(-0.3048, -0.3048); // (-0.263525, -0.263525);
    public static final Translation2d backRightLocation = new Translation2d(-0.3048, 0.3048); // (-0.263525, 0.263525);
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation);

    // uselses
    public static final double STATE_SPEED_THRESHOLD = 0.05;

    // The values of the can coders when the wheels are straight according to Mr. Wright
    public static final double CANCoderValue9 = 0.915283 + 0.5; // 0.9174805
    public static final double CANCoderValue10 = 0.327881; // 0.328613 + 0.5 add 0.5
    public static final double CANCoderValue11 = 0.979736 - 0.5; // 0.539794 - 0.5
    public static final double CANCoderValue12 = 0.536133; // 0.984863

    // THe deadband of the joystick to combat drift
    public static final double JOYSTICK_DEADBAND = 0.05;
    
    public static final boolean usingVision = false;
    public static final boolean isFieldOriented = true;

    // Whether the limelight auto aligns and its deadband
    public static final boolean useLimelightAutoAlign = true;
    public static final double limelightDeadband = 3.5;

    public static final double DEADBAND = 0.08;

    public static final int[] greenLED = { 60, 255, 255 };
    public static final int[] orangeLED = { 30, 255, 255 };
    public static final int[] redLED = { 0, 255, 255 };

    public static class BasePIDConstants {
      // public static final PID STEER_PID = new PID(0.14, 0.00002, 0.008, 0);
      public static final PID STEER_PID = new PID(0.15, 0.0, 0, 0); // 0.05 P, 0 D
      public static final PID DRIVE_PID = new PID(0.15, 0.0, 0, 0);
      // DON'T SET D PAST 0.03 - Erick or else the swerve moduls make funny nosie

      // AutoAlign PID
      public static final PID horizontalPID = new PID(0.05, 0.075, 0.03, 0);
      public static final PID verticalPID = new PID(0.25, 0.0085, 0.03);
      public static final PID rotationalPID = new PID(0.05, 0.003, 0.003, 0);

      // path planner things

      public static PIDController pathTranslationPID = new PIDController(0.15, 0.000, 0.00);
      public static PIDController pathRotationPID = new PIDController(2.0, 0.0, 0.0);

      public static HolonomicPathFollowerConfig pathFollower = new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig,
                                                                                                // this should likely
                                                                                                // live in your
                                                                                                // Constants class
          new PIDConstants(0.15, 0.000, 0.00), // translation
          new PIDConstants(0, 0.0, 0.0), // rotation
          4.96824, // Max module speed, in m/s
          SwerveConstants.robotSize, // Drive base radius in meters. Distance from robot center to furthest //
                                     // module.
          new ReplanningConfig(false, false)); // Default path replanning config. See the API for the options here
    }

    public static final double offBalanceAngleThreshold = 10;
    public static final double onBalanceAngleThreshold = 5;
  }

  public static class IntakeConstants {
    private IntakeConstants() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    public static final boolean isInverted = false;

    public static final int INTAKE_MOTOR_ID = 17;

    public static final double INTAKE_SPEED = 1000;


    public static final double INTAKE_PID_V = 0.1;
    public static final double INTAKE_PID_P = 0.0002;
    public static final double INTAKE_PID_I = 0.0;
    public static final double INTAKE_PID_D = 0.0;

    public static boolean HAS_PEICE = false;
  }

  public static class PivotConstants {
    private PivotConstants() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    public static final boolean isInverted = false;

    public static final int PIVOT_MOTOR_LEFT_ID = 101;
    public static final int PIVOT_MOTOR_RIGHT_ID = 102;

    public static final double PIVOT_PID_LEFT_P = 0.0002;
    public static final double PIVOT_PID_LEFT_I = 0.0;
    public static final double PIVOT_PID_LEFT_D = 0.0;

    public static final double PIVOT_PID_RIGHT_P = 0.0002;
    public static final double PIVOT_PID_RIGHT_I = 0.0;
    public static final double PIVOT_PID_RIGHT_D = 0.0;

    public static final double PIVOT_NEUTRAL_ANGLE = 0.0;
    public static final double PIVOT_AMP_ANGLE = 0.0;

    public static boolean IS_NEUTRAL = true;

    public static final int ENCODER_ID = 0;
  }

  public static class ShooterConstants {
    private ShooterConstants() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    public static final int FALCON_LEFT_ID = 103;
    public static final int FALCON_RIGHT_ID = 104;

    public static final int KRAKEN_ID = 105;

    public static final double SHOOTER_PID_LEFT_V = 0.1;
    public static final double SHOOTER_PID_LEFT_P = 0.0002;
    public static final double SHOOTER_PID_LEFT_I = 0.0;
    public static final double SHOOTER_PID_LEFT_D = 0.0;

    public static final double SHOOTER_PID_RIGHT_V = 0.1;
    public static final double SHOOTER_PID_RIGHT_P = 0.0002;
    public static final double SHOOTER_PID_RIGHT_I = 0.0;
    public static final double SHOOTER_PID_RIGHT_D = 0.0;

    public static final double KARKEN_V = 0.1;
    public static final double KRAKEN_P = 0.0002;
    public static final double KRAKEN_I = 0.0;
    public static final double KRAKEN_D = 0.0;

    public static final double SHOOTER_SPEED = 6942.0;
    public static final double KRAKEN_SPEED = 6942.0;

    public static boolean IS_SHOOTING = false;
  }
}