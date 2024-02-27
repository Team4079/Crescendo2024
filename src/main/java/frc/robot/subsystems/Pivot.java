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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Hell.PivotConstants;

public class Pivot extends SubsystemBase {
  /** Creates a new Intake. */

  private TalonFX pivotMotorLeft;
  private TalonFX pivotMotorRight;

  private TalonFXConfigurator pivotLeftConfigurator;
  private TalonFXConfigurator pivotRightConfigurator;

  private Slot0Configs pivotLeftConfigs;
  private Slot0Configs pivotRightConfigs;

  private VelocityVoltage m_request;

  private MotorOutputConfigs pivotConfigs;

  private CurrentLimitsConfigs leftMotorCurrentConfig;
  private CurrentLimitsConfigs rightMotorCurrentConfig;

  private ClosedLoopRampsConfigs leftMotorRampConfig;
  private ClosedLoopRampsConfigs rightMotorRampConfig;
  

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

    leftMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.5;
    rightMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.5;

    pivotMotorLeft.getConfigurator().apply(leftMotorRampConfig);
    pivotMotorRight.getConfigurator().apply(rightMotorRampConfig);

    m_request = new VelocityVoltage(0).withSlot(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotors() {
    pivotMotorLeft.stopMotor();
    pivotMotorRight.stopMotor();
  }

  public void setVelocity(double left, double right) {
    pivotMotorLeft.setControl(m_request.withVelocity(left));
    pivotMotorRight.setControl(m_request.withVelocity(right));
  }
}
