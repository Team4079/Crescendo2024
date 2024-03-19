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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.GlobalsValues;
import frc.robot.utils.GlobalsValues.MotorGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

/**
 * The {@link SwerveModule} class includes all the motors to control the swerve drive.
 */
public class SwerveModule {
  /** Creates a new SwerveModule. */
  public TalonFX driveMotor;
  public TalonFX steerMotor;
  public CANcoder canCoder;

  private MotorOutputConfigs motorConfigs;

  private TalonFXConfigurator driveConfigurator;
  private TalonFXConfigurator steerConfigurator;

  private Slot0Configs driveslot0Configs;
  private Slot0Configs steerslot0Configs;

  private PIDController steerPIDController;

  private DutyCycleOut m_request;

  private PositionDutyCycle m_cycle;
  private VelocityVoltage v_voltage;
  private double initialCANCoderValue;

  private final double CANCoderDriveStraightSteerSetPoint;

  private CurrentLimitsConfigs driveCurrentLimitsConfigs;
  private CurrentLimitsConfigs steerCurrentLimitsConfigs;

  private ClosedLoopRampsConfigs driveClosedRampsConfigs;
  private ClosedLoopRampsConfigs steerClosedRampsConfigs;

  private SwerveModuleState currentState;

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveId, int steerId, int canCoderID, double CANCoderDriveStraightSteerSetPoint) {
    driveMotor = new TalonFX(driveId);
    steerMotor = new TalonFX(steerId);
    canCoder = new CANcoder(canCoderID);

    this.CANCoderDriveStraightSteerSetPoint = CANCoderDriveStraightSteerSetPoint;

    motorConfigs = new MotorOutputConfigs();

    driveConfigurator = driveMotor.getConfigurator();
    steerConfigurator = steerMotor.getConfigurator();

    driveslot0Configs = new Slot0Configs();
    steerslot0Configs = new Slot0Configs();

    m_request = new DutyCycleOut(0);
    m_cycle = new PositionDutyCycle(0);
    v_voltage = new VelocityVoltage(0);

    TalonFXConfiguration driveMotorConfigs = new TalonFXConfiguration();
    driveConfigurator.refresh(driveMotorConfigs);
    driveMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
    driveMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
    driveMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    driveMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = SwerveGlobalValues.MOTOR_DEADBAND;
    driveMotorConfigs.CurrentLimits.SupplyCurrentLimit = 40;
    driveMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = false;
    driveMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 30;
    driveMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
    driveMotorConfigs.CurrentLimits.StatorCurrentLimit = 70;
    driveMotorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    driveMotorConfigs.Audio.AllowMusicDurDisable = true;
    driveConfigurator.apply(driveMotorConfigs);


    TalonFXConfiguration steerMotorConfigs = new TalonFXConfiguration();
    steerConfigurator.refresh(steerMotorConfigs);
    steerMotorConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerMotorConfigs.Feedback.FeedbackRemoteSensorID = canCoderID;
    steerMotorConfigs.Feedback.RotorToSensorRatio = MotorGlobalValues.STEER_MOTOR_GEAR_RATIO;
    steerMotorConfigs.Feedback.SensorToMechanismRatio = 1;
    steerMotorConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    steerMotorConfigs.Voltage.PeakForwardVoltage = 11.5;
    steerMotorConfigs.Voltage.PeakReverseVoltage = -11.5;
    steerMotorConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; //May want to change to break idk
    steerMotorConfigs.MotorOutput.DutyCycleNeutralDeadband = SwerveGlobalValues.MOTOR_DEADBAND;
    steerMotorConfigs.CurrentLimits.SupplyCurrentLimit = 30;
    steerMotorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerMotorConfigs.CurrentLimits.SupplyCurrentThreshold = 20;
    steerMotorConfigs.CurrentLimits.SupplyTimeThreshold = 0.25;
    steerMotorConfigs.Audio.AllowMusicDurDisable = true;
    steerConfigurator.apply(steerMotorConfigs);

    driveConfigurator.apply(motorConfigs);
    steerConfigurator.apply(motorConfigs);

    driveslot0Configs.kP = BasePIDGlobal.DRIVE_PID.p;
    driveslot0Configs.kI = BasePIDGlobal.DRIVE_PID.i;
    driveslot0Configs.kD = BasePIDGlobal.DRIVE_PID.d;

    steerPIDController = new PIDController(BasePIDGlobal.STEER_PID.p, BasePIDGlobal.STEER_PID.i, BasePIDGlobal.STEER_PID.d);
    steerPIDController.enableContinuousInput(0, 2 * Math.PI);
    steerPIDController.setTolerance(0.005);

    driveMotor.setInverted(SwerveGlobalValues.DRIVE_MOTOR_INVERETED);
    steerMotor.setInverted(SwerveGlobalValues.STEER_MOTOR_INVERTED);

    driveMotor.getConfigurator().apply(driveslot0Configs);
    steerMotor.getConfigurator().apply(steerslot0Configs);

    steerConfigurator.setPosition(CANCoderDriveStraightSteerSetPoint);

    driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
    steerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    driveClosedRampsConfigs = new ClosedLoopRampsConfigs();
    steerClosedRampsConfigs = new ClosedLoopRampsConfigs();

    driveMotor.getConfigurator().apply(driveCurrentLimitsConfigs);
    steerMotor.getConfigurator().apply(steerCurrentLimitsConfigs);

    currentState = new SwerveModuleState();
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
            encoderToAngle(steerMotor.getRotorPosition().getValue()
            )));
  }

  /**
   * Sets the speed of the drive motor.
   * 
   * @param speed The speed to set the drive motor to.
   * @return void
   */
  public void setDriveSpeed(double speed) {
    driveMotor.setControl(v_voltage.withVelocity(speed));
  }

  /**
   * Sets the speed of the steer motor.
   * 
   * @param speed The speed in RPM to set the steer motor to.
   * @return void
   */
  public void setSteerSpeed(double speed) {
    steerMotor.setControl(m_request.withOutput(speed));
  }

  /**
   * Sets the position of the steer motor.
   * 
   * @param rotations The rotations in degrees to set the steer motor to.
   * @return void
   */
  public void setSteerPosition(double degrees) {
    steerMotor.setControl(m_cycle.withPosition(degrees));
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
 * Converts normalized encoder values to degrees.
 * 
 * @param encoderValue The normalized encoder value (0 to 1).
 * @return double The encoder value in degrees.
 */
public double encoderToAngle(double encoderValue) {
  return Math.abs(encoderValue * 360) % 360;
}

  /**
   * Converts rotations to degrees.
   * 
   * @param rotations The wheel rotations to convert to degrees.
   * @param gearRatio The gear ratio of the motor.
   * @return double The rotations in degrees.
   */
  public double rotationsToAngle(double wheelRotations, double gearRatio) {
    return wheelRotations * 360 / gearRatio;
  }

  /**
   * Converts degrees to rotations.
   * 
   * @param angle     The angle to convert to rotations.
   * @param gearRatio The gear ratio of the motor.
   * @return double The angle in rotations.
   */
  public double angleToRotations(double angle, double gearRatio) {
    return (angle * MotorGlobalValues.ENCODER_COUNTS_PER_ROTATION) / (360 * gearRatio); 
  }

  /**
   * Converts degrees to encoder counts.
   * 
   * @param angle     The angle to convert to encoder counts.
   * @param gearRatio The gear ratio of the motor.
   * @return double The angle in encoder counts.
   */
  public double angleToEncoder(double angle, double gearRatio) {
    return angle * MotorGlobalValues.ENCODER_COUNTS_PER_ROTATION / 360 /
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
  public void setState(SwerveModuleState state) {
    state = SwerveModuleState.optimize(state,
      Rotation2d.fromDegrees(getCanCoderValueDegrees()));

    double desiredAngle = state.angle.getDegrees();
    double motor_Velocity = state.speedMetersPerSecond / MotorGlobalValues.MetersPerRevolution / MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO;
    
    double currentRotations = (steerMotor.getRotorPosition().getValue());
    Rotation2d currentAngle = Rotation2d
        .fromDegrees(rotationsToAngle(currentRotations, MotorGlobalValues.STEER_MOTOR_GEAR_RATIO));

    if (Math.abs(motor_Velocity) > 0.001) {
      setDriveSpeed(motor_Velocity);
    }

    double steer = steerPIDController.calculate(getCanCoderValueDegrees(), state.angle.getDegrees());
    setSteerSpeed(steer);

    // if (Math.abs(state.speedMetersPerSecond) > SwerveGlobalValues.STATE_SPEED_THRESHOLD) {
    //   double newRotations;
    //   Rotation2d delta = state.angle.minus(currentAngle);

    //   double change = delta.getDegrees();

    //   if (change > 90) {
    //     change -= 180;
    //   } else if (change < -90) {
    //     change += 180;
    //   }

      // newRotations = currentRotations + angleToRotations(change, MotorGlobalValues.STEER_MOTOR_GEAR_RATIO);
      // setSteerPosition(newRotations);
    // }
  }

  /**
   * Stops the module.
   * 
   * @param None
   * @return void
   */
  public void stop() {
    setDriveSpeed(0);
    setSteerSpeed(0);
  }

  /**
   * Sets the position of the steer motor to the current CANCoder value.
   * 
   * @param None
   * @return void
   */
  public void setRotorPos() {
    initialCANCoderValue = canCoder.getAbsolutePosition().refresh().getValue() % 360;
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
  public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInAppropriate0To360Scope(
        currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();

    if (Math.abs(delta) > 90) {
      targetSpeed = -targetSpeed;
      targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }
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
   * Returns the CANCoder value in degrees.
   * 
   * @param void
   * @return double The CANCoder value in degrees.
   */
  public double getCanCoderValueDegrees() {
    return ((360 * canCoder.getAbsolutePosition().getValue()) % 360 + 360) % 360;
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

  public double getDrivePosition() {
    return driveMotor.getRotorPosition().getValue()
    * MotorGlobalValues.MetersPerRevolution
    * MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO  ;
  }

      /**
     * Get the velocity of the drive motor
     * @return Velocity of the drive motor in m/s
     */
    public double getDriveVelocity() {
      return driveMotor.getRotorVelocity().getValue() 
          * MotorGlobalValues.MetersPerRevolution
          * MotorGlobalValues.DRIVE_MOTOR_GEAR_RATIO;
  }

  public double getSteerVelocity() {
    double turnVelocity = canCoder.getVelocity().getValue() * 360;
    return turnVelocity;
  }

  public SwerveModuleState getState() {
    currentState.angle = Rotation2d.fromDegrees(getCanCoderValueDegrees());
    currentState.speedMetersPerSecond = getDriveVelocity();
    return currentState;
  }

  public void addToSmartDashboard() {
    // Somebody pls add all the stuff to smartdashboard i dont want to do it -shawn
  }
}
