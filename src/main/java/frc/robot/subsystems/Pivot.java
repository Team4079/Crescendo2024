// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.dense.row.mult.SubmatrixOps_FDRM;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.PivotAlign;
import frc.robot.utils.Constants.PivotConstants;

/**
 * The {@link Pivot} class includes all the motors to pivot the shooter.
 * 
 *
 * 
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

  private MotorOutputConfigs pivotConfigs;

  private CurrentLimitsConfigs leftMotorCurrentConfig;
  private CurrentLimitsConfigs rightMotorCurrentConfig;

  private ClosedLoopRampsConfigs leftMotorRampConfig;
  private ClosedLoopRampsConfigs rightMotorRampConfig;

  private SparkAbsoluteEncoder absoluteEncoder;
  private CANSparkMax encoderController;

  private double absPos;

  public Pivot() {
    pivotMotorLeft = new TalonFX(PivotConstants.PIVOT_MOTOR_LEFT_ID);
    pivotMotorRight = new TalonFX(PivotConstants.PIVOT_MOTOR_RIGHT_ID);

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

    pivotLeftConfigs.kP = PivotConstants.PIVOT_PID_LEFT_P;
    pivotLeftConfigs.kI = PivotConstants.PIVOT_PID_LEFT_I;
    pivotLeftConfigs.kD = PivotConstants.PIVOT_PID_LEFT_D;

    pivotRightConfigs.kP = PivotConstants.PIVOT_PID_RIGHT_P;
    pivotRightConfigs.kI = PivotConstants.PIVOT_PID_RIGHT_I;
    pivotRightConfigs.kD = PivotConstants.PIVOT_PID_RIGHT_D;

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

    encoderController = new CANSparkMax(PivotConstants.ENCODER_ID, MotorType.kBrushless);
    absoluteEncoder = encoderController.getAbsoluteEncoder(Type.kDutyCycle);

    absoluteEncoder.setPositionConversionFactor(2048);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {

    absPos = absoluteEncoder.getPosition();
    
    
    SmartDashboard.putNumber("Absolute Encoder Position", getAbsoluteEncoder());
    SmartDashboard.putNumber("Pivot Left Position", pivotMotorLeft.getPosition().getValue());
    SmartDashboard.putNumber("Pivot Right Position", pivotMotorRight.getPosition().getValue());

    pivotMotorLeft.setPosition(absPos);
    pivotMotorRight.setPosition(absPos);
  }

  /**
   * Stops the pivot motors
   * @return void
   */
  public void stopMotors() {
    pivotMotorLeft.stopMotor();
    pivotMotorRight.stopMotor();
  }

  /**
   * Set the position of the left and right pivot motors
   * @param left Left motor position
   * @param right Right motor position
   * @return void
   */
  public void setMotorPosition(double left, double right) {
    pivotMotorLeft.setControl(pos_reqest.withPosition(left));
    pivotMotorRight.setControl(pos_reqest.withPosition(right));
  }

  /**
   * Get the position of the pivot motor
   * @return double, the position of the pivot motor
   */
  public double getPivotPos() {
    return pivotMotorLeft.getPosition().getValue();
  }

  /**
   * Run distance through a best fit line and return the value
   * @param distance
   * @return double, the position of the pivot motor
   */
  public double shootPos(double distance) {
    // line function
    // do stuf
    return 0.0;
  }

  public double getAbsoluteEncoder() {
    return absoluteEncoder.getPosition();
  }



 
}
