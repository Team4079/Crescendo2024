// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

/**
 * The {@link Pivot} class includes all the motors to pivot the shooter.
 */
public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */

  private TalonFX pivotMotorLeft;
  private TalonFX pivotMotorRight;

  private TalonFXConfigurator pivotLeftConfigurator;
  private TalonFXConfigurator pivotRightConfigurator;

  private Slot0Configs pivotLeftConfigs;
  private Slot0Configs pivotRightConfigs;

  private PositionVoltage pos_reqest;
  private VelocityVoltage vel_voltage;

  private MotorOutputConfigs pivotConfigs;

  private CurrentLimitsConfigs leftMotorCurrentConfig;
  private CurrentLimitsConfigs rightMotorCurrentConfig;

  private ClosedLoopRampsConfigs leftMotorRampConfig;
  private ClosedLoopRampsConfigs rightMotorRampConfig;

  // private SparkAbsoluteEncoder absoluteEncoder;
  DutyCycleEncoder actualAbsEnc;

  private DigitalInput absoluteEncoder;

  private double deadband = 0.05;

  private double absPos;

  public Pivot() {
    pivotMotorLeft = new TalonFX(PivotGlobalValues.PIVOT_MOTOR_LEFT_ID);
    pivotMotorRight = new TalonFX(PivotGlobalValues.PIVOT_MOTOR_RIGHT_ID);

    pivotConfigs = new MotorOutputConfigs();

    pivotLeftConfigurator = pivotMotorLeft.getConfigurator();
    pivotRightConfigurator = pivotMotorRight.getConfigurator();

    pivotLeftConfigs = new Slot0Configs();
    pivotRightConfigs = new Slot0Configs();

    pivotMotorLeft.getConfigurator().apply(new TalonFXConfiguration());
    pivotMotorRight.getConfigurator().apply(new TalonFXConfiguration());

    pivotConfigs.NeutralMode = NeutralModeValue.Brake;
    pivotLeftConfigurator.apply(pivotConfigs);
    pivotRightConfigurator.apply(pivotConfigs);

    pivotLeftConfigs.kP = PivotGlobalValues.PIVOT_PID_LEFT_P;
    pivotLeftConfigs.kI = PivotGlobalValues.PIVOT_PID_LEFT_I;
    pivotLeftConfigs.kD = PivotGlobalValues.PIVOT_PID_LEFT_D;
    pivotLeftConfigs.kV = PivotGlobalValues.PIVOT_PID_LEFT_V;
    // pivotLeftConfigs.kF = PivotGlobalValues.PIVOT_PID_LEFT_F;

    pivotRightConfigs.kP = PivotGlobalValues.PIVOT_PID_RIGHT_P;
    pivotRightConfigs.kI = PivotGlobalValues.PIVOT_PID_RIGHT_I;
    pivotRightConfigs.kD = PivotGlobalValues.PIVOT_PID_RIGHT_D;
    pivotRightConfigs.kV = PivotGlobalValues.PIVOT_PID_RIGHT_V;
    // pivotRightConfigs.kF = PivotGlobalValues.PIVOT_PID_RIGHT_F;

    pivotMotorLeft.getConfigurator().apply(pivotLeftConfigs);
    pivotMotorRight.getConfigurator().apply(pivotRightConfigs);

    leftMotorCurrentConfig = new CurrentLimitsConfigs();
    rightMotorCurrentConfig = new CurrentLimitsConfigs();

    leftMotorRampConfig = new ClosedLoopRampsConfigs();
    rightMotorRampConfig = new ClosedLoopRampsConfigs();

    leftMotorCurrentConfig.SupplyCurrentLimit = 100;
    leftMotorCurrentConfig.StatorCurrentLimit = 100;

    rightMotorCurrentConfig.SupplyCurrentLimit = 100;
    rightMotorCurrentConfig.StatorCurrentLimit = 100;

    pivotMotorLeft.getConfigurator().apply(leftMotorCurrentConfig);
    pivotMotorRight.getConfigurator().apply(rightMotorCurrentConfig);

    leftMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    rightMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    pivotMotorLeft.getConfigurator().apply(leftMotorRampConfig);
    pivotMotorRight.getConfigurator().apply(rightMotorRampConfig);

    // encoderController = new CANSparkMax(PivotGlobalValues.ENCODER_ID, CANSparkLowLevel.MotorType.kBrushless);
    // absoluteEncoder = encoderController.getAbsoluteEncoder(Type.kDutyCycle);

    // absoluteEncoder.setPositionConversionFactor(2048);

    absoluteEncoder = new DigitalInput(9);
    // encoder = new AbsoluteEncoder(8);

    actualAbsEnc = new DutyCycleEncoder(absoluteEncoder);

    vel_voltage = new VelocityVoltage(0);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // 0.545533063638327 High limit before 2048 mulitplier
    // 0.201015130025378 Low limit before 2048 multiplier
    // absPos = absoluteEncoder.getPosition();

    SmartDashboard.putNumber("Absolute Encoder Position", getAbsoluteEncoder());
    SmartDashboard.putNumber("Pivot Left Position", pivotMotorLeft.getPosition().getValue());
    SmartDashboard.putNumber("Pivot Right Position", pivotMotorRight.getPosition().getValue());

    pivotMotorLeft.setPosition(absPos);
    pivotMotorRight.setPosition(absPos);

    if (absPos == PivotGlobalValues.PIVOT_NEUTRAL_ANGLE) {
      GlobalsValues.PivotGlobalValues.IS_NEUTRAL = true;
    }
  }

  /**
   * Stops the pivot motors
   * 
   * @param void
   * @return void
   */
  public void stopMotors() {
    pivotMotorLeft.stopMotor();
    pivotMotorRight.stopMotor();
  }

  /**
   * Set the position of the left and right pivot motors
   * 
   * @param left  Left motor position
   * @param right Right motor position
   * @return void
   */
  public void setMotorPosition(double left, double right) {
    pivotMotorLeft.setControl(pos_reqest.withPosition(left));
    pivotMotorRight.setControl(pos_reqest.withPosition(right));
  }

  /**
   * Get the position of the pivot motor
   * 
   * @param void
   * @return double, the position of the pivot motor
   */
  public double getPivotPos() {
    return pivotMotorLeft.getPosition().getValue();
  }

  /**
   * Run distance through a best fit line and return the value
   * 
   * @param distance
   * @return double, the position of the pivot motor
   */
  public double shootPos(double distance) {
    // line function
    // do stuf
    return 0.0;
  }

  /**
   * Get the absolute encoder position
   * 
   * @param void
   * @return double, the absolute encoder position of the pivot motor
   */
  public double getAbsoluteEncoder() {
    // return 0;
    return actualAbsEnc.getAbsolutePosition();
  }

  /**
   * Zeros the absolute encoder
   * 
   * @param void
   * @return void
   */
  public void zeroAbsoluteEncoder() {
    
  }

  public void movePivot(double velocity) {
    if (Math.abs(velocity) >= deadband) {
      pivotMotorLeft.setControl(vel_voltage.withVelocity(velocity * 500));
      pivotMotorRight.setControl(vel_voltage.withVelocity(velocity * 500));
    } else {
      stopMotors();
    }
  }
}