// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class GlobalsValues {
  private static final String UTILITY_CLASS = "Utility class";

  private GlobalsValues() {
    throw new IllegalStateException(UTILITY_CLASS);
  }

  public static class MotorGlobalValues {
    private MotorGlobalValues() {
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

    public static final int PIDGEY_ID = 16;

    // Motor Property Values
    public static final double MAX_SPEED = 5.76;
    public static final double MAX_ANGULAR_SPEED = (14 * Math.PI) / 3;
    public static final double ENCODER_COUNTS_PER_ROTATION = 1; // 2048 for v5, 1 for v6 (rotations)
    public static final double STEER_MOTOR_GEAR_RATIO = 150.0 / 7; // 24
    public static final double DRIVE_MOTOR_GEAR_RATIO = 5.9;
    public static final double WHEEL_DIAMETER = 0.106;
    public static final double SPEED_CONSTANT = 0.6; // 0.4
    public static final double AACORN_SPEED = 0.95;
    public static final double SLOW_SPEED = 0.3;
    public static final double TURN_CONSTANT = 0.3; // 0.3
    public static final double MetersPerRevolution = WHEEL_DIAMETER * Math.PI;
    public static double HEADING = 0.0;

    // Limit Values
    public static final double DRIVE_SUPPLY_LIMIT = 45;
    public static final double DRIVE_STATOR_LIMIT = 80;
    public static final double DRIVE_SUPPLY_THRESHOLD = 30;
    public static final double DRIVE_TIME_THRESHOLD = 0.25;
    public static final double STEER_SUPPLY_LIMIT = 30;
    public static final double STEER_SUPPLY_THRESHOLD = 30;
    public static final double STEER_TIME_THRESHOLD = 0.25;

    // Motor Speed Manipulation Values
    public static boolean SLOW_MODE = false;
    public static boolean AACORN_MODE = true;
  }

  public static class ElevatorGlobalValues {
    public static final int ELEVATOR_NEO_ID = 21;
    public static final int PASS_NEO_ID = 22;
    public static final double closedLoopRampRate = 0.5;
    public static final double passClosedLoopRampRate = 0.1;
    public static final double ELEVATOR_UP = 7; // TODO: git gud
    public static final double ELEVATOR_DOWN = 0.5;
    public static final double ELEVATOR_CLIMB = 7.4;
    public static final double kP = 0.000045; // TODO: git gud
    public static final double kI = 0.000000; // TODO: git gud
    public static final double kD = 0.0; // TODO: git gud
    public static final double kIz = 0.0;
    public static final double kFF = 0.0002;
    public static final double kMaxOutput = 0.8;
    public static final double kMinOutput = -0.1;

    public static final double PasskP = 0.001; // TODO: git gud
    public static final double PasskI = 0; // TODO: git gud
    public static final double PasskD = 0; // TODO: git gud
    public static final float SOFTLIMIT_FOWARD = 7.5f;
    public static final float SOFTLIMIT_REVERSE = -1.0f;

    public static final boolean ELEVATOR_JOYSTICKS = false;
  }

  public static class SwerveGlobalValues {
    private SwerveGlobalValues() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    public static final double ROBOT_SIZE = 0.43105229381; // Keep constant *ideally*

    // Motor Locations (Relative to the center in meters)
    public static final Translation2d FRONT_LEFT =
        new Translation2d(0.3048, 0.3048); // (0.263525, -0.263525);
    public static final Translation2d FRONT_RIGHT =
        new Translation2d(0.3048, -0.3048); // (0.263525, 0.263525);
    public static final Translation2d BACK_LEFT =
        new Translation2d(-0.3048, 0.3048); // (-0.263525, -0.263525);
    public static final Translation2d BACK_RIGHT =
        new Translation2d(-0.3048, -0.3048); // (-0.263525, 0.263525);
    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);

    public static final double BASE_LENGTH_ERICK_TRAN = 0.3048 * 2;

    // uselses
    public static final double STATE_SPEED_THRESHOLD = 0.05; 

    // The values of the can coders when the wheels are straight according to Mr.
    // Wright
    public static final double CANCoderValue9 = -0.419189; // 0.4198189
    public static final double CANCoderValue10 = -0.825928 - 0.5; // 0.328613 + 0.5 add 0.5
    public static final double CANCoderValue11 = -0.475098; // 0.539794 - 0.5%
    public static final double CANCoderValue12 = -0.032959 + 0.5; // 0.984863

    // Whether the motors are inverted
    public static final InvertedValue DRIVE_MOTOR_INVERETED =
        InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue STEER_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;

    // THe deadband of the joystick to combat drift
    public static final double JOYSTICK_DEADBAND = 0.05;

    public static final boolean USING_VISION = false;
    public static final boolean FIELD_ORIENTATED = true;

    // Whether the limelight auto aligns and its deadband
    public static final boolean AUTO_ALIGN = false;
    public static final double LIMELIGHT_DEADBAND = 1.5;

    public static final double MOTOR_DEADBAND = 0.05;
    public static final boolean IS_FIELD_ORIENTATED = true;
    public static final double ENCODER_OFFSET =
        0 / 360; // TODO add an offset for the canCoder getting the position at the beginning

    public static Pose2d currentPose = new Pose2d(0, 0, new Rotation2d(0));

    // RGB Values for LED
    public static final int[] GREEN_LED = {0, 255, 0};
    public static final int[] ORANGE_LED = {255, 165, 0};
    public static final int[] HIGHTIDE_LED = {0, 182, 174};

    public static class BasePIDGlobal {
      public static final PID STEER_PID_TELE = new PID(13, 0.000, 0.1, 0);
      public static final PID STEER_PID_AUTO = new PID(15, 0.000, 0.1, 0);
      // public static final PID STEER_PID = new PID(0.15, 0.0000, 0.000005, 0);
      // 0.05 P, 0 D

      public static final PID DRIVE_PID_AUTO = new PID(7, 0.0, 0.00); //0.00031
      public static final double DRIVE_PID_V_AUTO = 0.5;

      // public static final PID DRIVE_PID_TELE = new PID(0.0005, 0.0, 0.00); //0.00031
      // public static final double DRIVE_PID_V_TELE = 1;
      public static final PID DRIVE_PID_TELE = new PID(1, 0.0, 0.0);
      public static final double DRIVE_PID_V_TELE = 0;
      // DON'T SET D PAST 0.03 - Erick or else the swerve moduls make funny nosie

      // AutoAlign PID
      public static final PID ROTATIONAL_PID = new PID(0.15, 0.0, 0, 0);
      public static final PID PASS_ROTATIONAL_PID = new PID(0.075, 0000, 0.00, 0);

      // PathPlanner Push Variable Turn smartdasboard on or off
      public static boolean TEST_MODE = false;

      public static HolonomicPathFollowerConfig pathFollower =
      new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0.00, 0), // translation
        new PIDConstants(5, 0.0, 0), // rotation
        4, // Max module speed, in m/s
        SwerveGlobalValues
            .ROBOT_SIZE, // Drive base radius in meters. Distance from robot center to
        // furthest //
        // module.
        new ReplanningConfig(
            false,
            false)); // Default path replanning config. See the API for the options here
    }

    // Controller X and Y deadbands
    public static final double xDEADZONE = 0.15 * 5.76;
    public static final double yDEADZONE = 0.15 * 5.76;

    public static final double offBalanceAngleThreshold = 10;
    public static final double onBalanceAngleThreshold = 5;
  }

  public static class IntakeGlobalValues {
    private IntakeGlobalValues() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    // Intake Motor Values
    public static final boolean isInverted = false;

    public static final int INTAKE_MOTOR_ID = 17;

    public static final double INTAKE_SPEED = 60;

    // Intake PID Values
    public static final double INTAKE_PID_V = 0.1;
    public static final double INTAKE_PID_P = 0.0002;
    public static final double INTAKE_PID_I = 0.0;
    public static final double INTAKE_PID_D = 0.0;

    // Reverse Intake Values
    public static final double REVERSE_INTAKE_SPEED = 20.0;
  }

  public static class PivotGlobalValues {
    private PivotGlobalValues() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    // Pivot Motor Values
    public static final boolean IS_INVERTED = false;
    public static final double PIVOT_GEAR_RATIO = 198.0;

    // Pivot Motor CAN ID Values
    public static final int PIVOT_MOTOR_LEFT_ID = 14;
    public static final int PIVOT_MOTOR_RIGHT_ID = 15;

    // Pivot PID Values
    public static final double PIVOT_PID_LEFT_P = 0.3;
    public static final double PIVOT_PID_LEFT_I = 0.0005;
    public static final double PIVOT_PID_LEFT_D = 0.0;
    public static final double PIVOT_PID_LEFT_V = 0.5;
    public static final double PIVOT_PID_LEFT_F = 0.1;

    public static final double PIVOT_PID_RIGHT_P = 0.3;
    public static final double PIVOT_PID_RIGHT_I = 0.0005;
    public static final double PIVOT_PID_RIGHT_D = 0.0;
    public static final double PIVOT_PID_RIGHT_V = 0.5;
    public static final double PIVOT_PID_RIGHT_F = 0.1;

    // SetPivot PID Controller
    public static final double SETPIVOT_PID_P = 0.00825;
    public static final double SETPIVOT_PID_I = 0;
    public static final double SETPIVOT_PID_D = 0.00035;

    public static final double PIVOT_NEUTRAL_ANGLE = (0.5 + PivotGlobalValues.offset); // All 3\
    // public static final double PIVOT_AMP_ANGLE = (46 + PivotGlobalValues.offset);
    // //Ued to be 49.55
    public static final double PIVOT_AMP_ANGLE = 19;
    public static final double PIVOT_SUBWOOFER_ANGLE = (13.4 + PivotGlobalValues.offset);
    // public static final double PIVOT_SOURCE = 93;
    public static final double PIVOT_FENDER_ANGLE =
        (30.5 + PivotGlobalValues.offset); // 305 instead of 30.5
    // public static final double PIVOT_MAX_ANGLE = (55 + PivotGlobalValues.offset);
    // public static final double PIVOT_MIN_ANGLE = (0 + PivotGlobalValues.offset);

    public static final double PIVOT_PASS_ANGLE = (PivotGlobalValues.offset);

    // Pivot Motor Speed Values
    public static boolean IS_NEUTRAL = true;
    public static boolean soft_limit_enabled = true;

    // Pivot Motor Encoder ID
    public static final int ENCODER_ID = 9;

    // Pivot Softlimit Values
    public static boolean is_SOFTLIMIT = true;
    public static double offset = 0;
    public static double manual_offset = 1045;
  }

  public static class ShooterGlobalValues {
    private ShooterGlobalValues() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    // Shooter CAN ID Values
    public static final int FALCON_LEFT_ID = 18;
    public static final int FALCON_RIGHT_ID = 13;

    public static final int KRAKEN_ID = 20;

    // Shooter PID Values
    public static final double SHOOTER_PID_LEFT_P = 0.0002;
    public static final double SHOOTER_PID_LEFT_I = 0.0;
    public static final double SHOOTER_PID_LEFT_D = 0.0;
    public static final double SHOOTER_PID_LEFT_V = 0.5;

    public static final double SHOOTER_PID_RIGHT_P = 0.0002;
    public static final double SHOOTER_PID_RIGHT_I = 0.0;
    public static final double SHOOTER_PID_RIGHT_D = 0.0;
    public static final double SHOOTER_PID_RIGHT_V = 0.5;

    public static final double KRAKEN_P = 0.0002;
    public static final double KRAKEN_I = 0.0;
    public static final double KRAKEN_D = 0.0;
    public static final double KRAKEN_V = 0.1;

    // Shooter Motor Speed Values
    public static final double SHOOTER_SPEED = 30.0;
    public static double AMP_SPEED = 5;
    public static boolean AMP_SPEED_JESSICA = true;
    public static final double KRAKEN_SPEED = 15.0;

    // Shooter Motor Speeds
    public static final double SHOOTER_RPS = -40.0;
    public static final double SHOOTER_PASS = -8.0; // Positive is shooting, negative is intake
    public static final double STAGE_PASS_RPS = -15.0;
    public static final double PASSTHROUGH_RPS = -30.0;
    public static final double PUSH_UP_RPS = -10.0; // Positive is shooting, negative is intake
    public static final double AUTO_PASSTHROUGH_RPS =
        -35.0; // Positive is shooting, negative is intake
    public static final double PUSH_RING_AMP_RPS = -30;
    public static final double PUSH_RING_RPS = -30;

    // Shooter Misc Values
    public static boolean HAS_PIECE = false;
    public static double SHOOTING_DEADBAND = 0.3;
    public static double RPM_THRESHOLD = 25;

    public static final int RING_SENSOR_PORT = 8;

    // Side Angles
    public static final double blueSideAngle = -25.0;
    public static final double redSideAngle = 25.0;
  }

  public static class LimelightGlobalValues {
    private LimelightGlobalValues() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    // Offset Values
    public static double tx = 0.0;
    public static double ty = 0.0;
    public static double ta = 0.0;
    public static double tv = 0.0;

    // Limelight Misc Values
    // Limelihgt is 30 degrees
    public static double[] robotPoseTargetSpace = new double[6];
    public static double tagIDAvailable = 0.0;

    public static boolean hasTarget = false;

    public static double distance = 0;
  }

  public static class PhotonVisionConstants {
    private PhotonVisionConstants() {
      throw new IllegalStateException(UTILITY_CLASS);
    }

    // Offset Values
    public static double tx = 0.0;
    public static double ty = 0.0;
    public static double ta = 0.0;
    public static double tv = 0.0;

    // Camera One
    public static final double CAMERA_ONE_HEIGHT_METER = 0.47;
    public static final double CAMERA_ONE_ANGLE_DEG = 33; // up is positive
    public static final double OFFSET_TOWARD_MID_LEFT = -15.00;

    // Camera Two
    public static final double CAMERA_TWO_HEIGHT_METER = 0.61;
    public static final double CAMERA_TWO_ANGLE_DEG = 37.5; // up is positive
    public static final double OFFSET_TOWARD_MID_RIGHT = 15.0;
  }
}
