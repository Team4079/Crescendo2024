// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.GlobalsValues.MotorGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

/**
 * The {@link SwerveModule} class includes all the motors to control the swerve drive.
 */
public class SwerveModule {
  /** Creates a new SwerveModule. */
  private TalonFX driveMotor;
  private TalonFX steerMotor;
  private CANcoder canCoder;

  private MotorOutputConfigs motorConfigs;

  private TalonFXConfigurator driveConfigurator;
  private TalonFXConfigurator steerConfigurator;

  private Slot0Configs driveslot0Configs;
  private Slot0Configs steerslot0Configs;

  private DutyCycleOut dutyCycle;

  private PositionDutyCycle cycle;

  private final double CANCoderDriveStraightSteerSetPoint;

  private CurrentLimitsConfigs driveCurrentLimitsConfigs;
  private CurrentLimitsConfigs steerCurrentLimitsConfigs;

  private ClosedLoopRampsConfigs driveClosedRampsConfigs;
  private ClosedLoopRampsConfigs steerClosedRampsConfigs;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int steerId, int canCoderID, double CANCoderDriveStraightSteerSetPoint) {
    setDriveMotor(new TalonFX(driveId));
    setSteerMotor(new TalonFX(steerId));
    setCanCoder(new CANcoder(canCoderID));

    this.CANCoderDriveStraightSteerSetPoint = CANCoderDriveStraightSteerSetPoint;

    motorConfigs = new MotorOutputConfigs();

    driveConfigurator = driveMotor.getConfigurator();
    steerConfigurator = steerMotor.getConfigurator();

    driveslot0Configs = new Slot0Configs();
    steerslot0Configs = new Slot0Configs();

    dutyCycle = new DutyCycleOut(0);
    cycle = new PositionDutyCycle(0);

    driveMotor.getConfigurator().apply(new TalonFXConfiguration());
    steerMotor.getConfigurator().apply(new TalonFXConfiguration());

    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    driveConfigurator.apply(motorConfigs);
    steerConfigurator.apply(motorConfigs);

    driveslot0Configs.kP = BasePIDGlobal.DRIVE_PID.p;
    driveslot0Configs.kI = BasePIDGlobal.DRIVE_PID.i;
    driveslot0Configs.kD = BasePIDGlobal.DRIVE_PID.d;

    steerslot0Configs.kP = BasePIDGlobal.STEER_PID.p;
    steerslot0Configs.kI = BasePIDGlobal.STEER_PID.i;
    steerslot0Configs.kD = BasePIDGlobal.STEER_PID.d;

    driveMotor.getConfigurator().apply(driveslot0Configs);
    steerMotor.getConfigurator().apply(steerslot0Configs);

    steerConfigurator.setPosition(CANCoderDriveStraightSteerSetPoint);
    driveConfigurator.setPosition(0);

    driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
    steerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    driveClosedRampsConfigs = new ClosedLoopRampsConfigs();
    steerClosedRampsConfigs = new ClosedLoopRampsConfigs();

    driveClosedRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.1;
    steerClosedRampsConfigs.DutyCycleClosedLoopRampPeriod = 0.05;

    driveMotor.getConfigurator().apply(driveCurrentLimitsConfigs);
    steerMotor.getConfigurator().apply(steerCurrentLimitsConfigs);

    steerMotor.setInverted(SwerveGlobalValues.STEER_MOTOR_INVERTED);
  }

  /**
   * Returns the current position of the module.
   * 
   * @param None
   * @return SwerveModulePosition The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        encoderToMeters(
            driveMotor.getRotorPosition().getValue(), MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO),
        Rotation2d.fromDegrees(
            encoderToAngle(steerMotor.getRotorPosition().getValue(),
                MotorGlobalValues.STEER_MOTOR_GEAR_RATIO)));
  }

  /**
   * Sets the speed of the drive motor.
   * 
   * @param speed The speed to set the drive motor to.
   * @return void
   */
  public void setDriveSpeed(double speed) {
    driveMotor.setControl(dutyCycle.withOutput(speed));
  }

  /**
   * Sets the speed of the steer motor.
   * 
   * @param speed The speed in RPM to set the steer motor to.
   * @return void
   */
  public void setSteerSpeed(double speed) {
    steerMotor.setControl(dutyCycle.withOutput(speed));
  }

  /**
   * Sets the position of the steer motor.
   * 
   * @param rotations The rotations in degrees to set the steer motor to.
   * @return void
   */
  public void setSteerPosition(double degrees) {
    steerMotor.setControl(cycle.withPosition(degrees));
  }

  /**
   * Resets the drive motor encoder.
   * 
   * @param void
   * @return void
   */
  public void resetEncoders() {
    driveMotor.setPosition(0);
  }

  /**
   * Converts encoder counts to degrees.
   * 
   * @param encoderCount The encoder count to convert to degrees.
   * @param gearRatio    The gear ratio of the motor.
   * @return double The encoder count in degrees.
   */
  public double encoderToAngle(double encoderCount, double gearRatio) {
    return encoderCount * 360 /
        (MotorGlobalValues.ENCODER_COUNTS_PER_ROTATION * gearRatio);
  }

  /**
   * Converts rotor rotations to wheel degrees.
   * 
   * @param rotations The rotor rotations to convert to degrees.
   * @param gearRatio The gear ratio of the motor.
   * @return double The wheel rotations in degrees.
   */
  public double rotationsToAngle(double rotorRotations, double gearRatio) {
    return rotorRotations * 360 / gearRatio;
  }

  /**
   * Converts degrees to rotor rotations.
   * 
   * @param angle     The angle to convert to rotor rotations.
   * @param gearRatio The gear ratio of the motor.
   * @return double The number of rotor rotations.
   */
  public double angleToRotations(double angle, double gearRatio) {
    return angle / 360 * gearRatio;
  }

  /**
   * Converts degrees to encoder counts.
   * 
   * @param angle     The angle to convert to encoder counts.
   * @param gearRatio The gear ratio of the motor.
   * @return double The angle in encoder counts.
   */
  public double angleToEncoder(double angle, double gearRatio) {
    return angle * MotorGlobalValues.ENCODER_COUNTS_PER_ROTATION / 360 *
        gearRatio;
  }

  /**
   * Converts encoder counts to meters.
   * 
   * @param encoderCount The encoder count to convert to meters.
   * @param gearRatio    The gear ratio of the motor.
   * @return double The encoder count in meters.
   */
  public double encoderToMeters(double encoderCount, double gearRatio) {
    return encoderCount / (MotorGlobalValues.ENCODER_COUNTS_PER_ROTATION *
        gearRatio) * MotorGlobalValues.WHEEL_DIAMETER * Math.PI;
  }

  /**
   * Converts meters to encoder counts.
   * 
   * @param meters    double The meters to convert to encoder counts.
   * @param gearRatio double The gear ratio of the motor.
   * @return double The meters in encoder counts.
   */
  public double metersToEncoder(double meters, double gearRatio) {
    return meters / (MotorGlobalValues.WHEEL_DIAMETER * Math.PI) *
        MotorGlobalValues.ENCODER_COUNTS_PER_ROTATION * gearRatio;
  }

  /**
   * Sets the state of the module.
   * 
   * @param state SwerveModuleState The state to set the module to.
   * @return void
   */
  public void setState(SwerveModuleState state, int i) {

    SmartDashboard.putNumber("state 1 of " + i,  state.angle.getDegrees());

    // state = SwerveModule.optimize(state,
    //     Rotation2d.fromDegrees(
    //         rotationsToAngle(steerMotor.getRotorPosition().getValue(), MotorGlobalValues.STEER_MOTOR_GEAR_RATIO)),
    //     steerMotor.getDeviceID());

    SmartDashboard.putNumber("state 2 of " + i,  state.angle.getDegrees() % 360);

    double currentRotations = (steerMotor.getRotorPosition().getValue());
    Rotation2d currentAngle = Rotation2d
        .fromDegrees(rotationsToAngle(currentRotations, MotorGlobalValues.STEER_MOTOR_GEAR_RATIO));

    SmartDashboard.putNumber("current angle at point 2 of " + i, currentAngle.getDegrees() % 360);

    setDriveSpeed(state.speedMetersPerSecond / MotorGlobalValues.MAX_SPEED);

    if (Math.abs(state.speedMetersPerSecond) > SwerveGlobalValues.STATE_SPEED_THRESHOLD) {
      double newRotations;

      Rotation2d delta = state.angle.minus(currentAngle);

      // Problem: current angle is from 0-360
      // state angle is from -180 to 180
      // values cannot be subtracted as different values
      SmartDashboard.putNumber("state angle", state.angle.getDegrees() % 360);
      SmartDashboard.putNumber("currentAngle", currentAngle.getDegrees() % 360);

      double change = delta.getDegrees();

      SmartDashboard.putNumber("change at point 3 of " + i, change);

      // end 

      if (change > 90) {
        change -= 180;
      } else if (change < -90) {
        change += 180;
      }

      SmartDashboard.putNumber("change atfter point 3 of " + i, change);

      newRotations = currentRotations + angleToRotations(change, MotorGlobalValues.STEER_MOTOR_GEAR_RATIO);

      SmartDashboard.putNumber("new rotations at point 4 of " + i, newRotations % 360);
      // SmartDashboard.putNumber("Set Rotations " + steerMotor.getDeviceID(), newRotations);
      // SmartDashboard.putNumber("Actual Rotations " + steerMotor.getDeviceID(),
      //     steerMotor.getRotorPosition().getValue());
          // newRotations - steerMotor.getRotorPosition().getValue());
      setSteerPosition(newRotations);
    }
  }

  /**
   * Stops the module.
   * 
   * @return void
   */
  public void stop() {
    driveMotor.stopMotor();
    steerMotor.stopMotor();
  }

  /**
   * Sets the position of the steer motor to the current CANCoder value.
   * 
   * @return void
   */
  public void setRotorPos() {
    double initialCANCoderValue;
    initialCANCoderValue = canCoder.getAbsolutePosition().refresh().getValue();
    steerMotor.setPosition(
        -(initialCANCoderValue - CANCoderDriveStraightSteerSetPoint) * MotorGlobalValues.STEER_MOTOR_GEAR_RATIO);
  }

  /**
   * The function optimizes the desired rotation of the module.
   * The module will rotate relative to:
   * 1. Robot-relative: The front of the robot
   * 2. Field-relative: Wherever we zeroed the gyro
   * 
   * @param desiredState SwerveModuleState The desired state of the module.
   * @param currentAngle Rotation2d The current angle of the module.
   * @param deviceID     int The device ID of the module.
   * @return SwerveModuleState The optimized state of the module.
   */
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle, int deviceID) {
    double targetAngle = placeInAppropriate0To360Scope(
        currentAngle.getDegrees(), desiredState.angle.getDegrees());

    SmartDashboard.putNumber("targetAngle faster 360 change", targetAngle);
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();

    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? targetAngle - 180 : targetAngle + 180;
    }

    SmartDashboard.putNumber(deviceID + " Motor", currentAngle.getDegrees() - targetAngle);
    return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  /**
   * Rotates the angle to the appropriate direction in the 0 to 360 scope.
   * Updates angle to be within 0 to 360 degrees.
   * 
   * @param scopeReference double The reference angle to rotate the new angle to.
   * @param newAngle       double The new angle to rotate to the reference angle.
   * @return double The new angle in the 0 to 360 scope.
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;

    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }

  /**
   * Returns the CANCoder value.
   * 
   * @param void
   * @return double The CANCoder value.
   */
  public double getCanCoderValue() {
    return canCoder.getAbsolutePosition().getValue();
  }

  /**
   * Returns the current position of the steer motor.
   * 
   * @param void
   * @return double The current position of the steer motor.
   */
  public double getRotationDegree() {
    return rotationsToAngle(steerMotor.getRotorPosition().getValue(), MotorGlobalValues.STEER_MOTOR_GEAR_RATIO);
  }

  public double getEncoderCount() {
    return steerMotor.getRotorPosition().getValue();
  }

  public double getDriveVelocity() {
    return driveMotor.getRotorVelocity().getValue() / MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO;
  }

  public SwerveModuleState getState() {
    SwerveModuleState currentState = new SwerveModuleState();
    currentState.angle = Rotation2d
        .fromDegrees(rotationsToAngle(steerMotor.getRotorPosition().getValue(), MotorGlobalValues.STEER_MOTOR_GEAR_RATIO));
    currentState.speedMetersPerSecond = getDriveVelocity();
    return currentState;
  }

  public TalonFX getDriveMotor() {
    return driveMotor;
  }

  public void setDriveMotor(TalonFX driveMotor) {
    this.driveMotor = driveMotor;
  }

  public TalonFX getSteerMotor() {
    return steerMotor;
  }

  public void setSteerMotor(TalonFX steerMotor) {
    this.steerMotor = steerMotor;
  }

  public CANcoder getCanCoder() {
    return canCoder;
  }

  public void setCanCoder(CANcoder canCoder) {
    this.canCoder = canCoder;
  }
}