// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

/** The {@link Pivot} class includes all the motors to pivot the shooter. */
public class Pivot extends SubsystemBase {
  /** Creates a new Pivot. */
  private TalonFX pivotMotorLeft;

  private TalonFX pivotMotorRight;

  private TalonFXConfigurator pivotLeftConfigurator;
  private TalonFXConfigurator pivotRightConfigurator;

  private TalonFXConfiguration pivotLeftConfiguration;
  private TalonFXConfiguration pivotRightConfiguration;

  private Slot0Configs pivotLeftConfigs;
  private Slot0Configs pivotRightConfigs;

  private PositionVoltage pos_reqest;
  private VelocityVoltage vel_voltage;

  private MotorOutputConfigs pivotConfigs;

  private CurrentLimitsConfigs leftMotorCurrentConfig;
  private CurrentLimitsConfigs rightMotorCurrentConfig;

  private ClosedLoopRampsConfigs leftMotorRampConfig;
  private ClosedLoopRampsConfigs rightMotorRampConfig;

  private SoftwareLimitSwitchConfigs leftSoftLimitConfig;
  private SoftwareLimitSwitchConfigs rightSoftLimitConfig;

  private VoltageOut voltageOut;

  private double deadband = 0.001;

  private double absPos;

  public Pivot() {
    pivotMotorLeft = new TalonFX(PivotGlobalValues.PIVOT_MOTOR_LEFT_ID);
    pivotMotorRight = new TalonFX(PivotGlobalValues.PIVOT_MOTOR_RIGHT_ID);

    pivotConfigs = new MotorOutputConfigs();

    pivotLeftConfigurator = pivotMotorLeft.getConfigurator();
    pivotRightConfigurator = pivotMotorRight.getConfigurator();

    pivotLeftConfigs = new Slot0Configs();
    pivotRightConfigs = new Slot0Configs();

    pivotLeftConfiguration = new TalonFXConfiguration();
    pivotRightConfiguration = new TalonFXConfiguration();

    pivotMotorLeft.getConfigurator().apply(pivotLeftConfiguration);
    pivotMotorRight.getConfigurator().apply(pivotRightConfiguration);

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

    leftSoftLimitConfig = new SoftwareLimitSwitchConfigs();
    rightSoftLimitConfig = new SoftwareLimitSwitchConfigs();

    leftMotorCurrentConfig.SupplyCurrentLimit = 100;
    leftMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    leftMotorCurrentConfig.StatorCurrentLimit = 100;
    leftMotorCurrentConfig.StatorCurrentLimitEnable = true;

    rightMotorCurrentConfig.SupplyCurrentLimit = 100;
    rightMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    rightMotorCurrentConfig.StatorCurrentLimit = 100;
    rightMotorCurrentConfig.StatorCurrentLimitEnable = true;

    pivotMotorLeft.getConfigurator().apply(leftMotorCurrentConfig);
    pivotMotorRight.getConfigurator().apply(rightMotorCurrentConfig);

    leftMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    rightMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    pivotMotorLeft.getConfigurator().apply(leftMotorRampConfig);
    pivotMotorRight.getConfigurator().apply(rightMotorRampConfig);

    // on
    leftSoftLimitConfig.ForwardSoftLimitEnable = true;
    leftSoftLimitConfig.ReverseSoftLimitEnable = true;
    leftSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    leftSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    rightSoftLimitConfig.ForwardSoftLimitEnable = true;
    rightSoftLimitConfig.ReverseSoftLimitEnable = true;
    rightSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    pivotMotorLeft.setInverted(true);
    pivotMotorRight.setInverted(true);

    // off
    // leftSoftLimitConfig.ForwardSoftLimitEnable = false;
    // leftSoftLimitConfig.ReverseSoftLimitEnable = false;

    // rightSoftLimitConfig.ForwardSoftLimitEnable = false;
    // rightSoftLimitConfig.ReverseSoftLimitEnable = false;

    pivotLeftConfiguration.SoftwareLimitSwitch = leftSoftLimitConfig;
    pivotRightConfiguration.SoftwareLimitSwitch = rightSoftLimitConfig;

    pivotMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    pivotMotorRight.getConfigurator().apply(rightSoftLimitConfig);

    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityVoltage(0);
    pos_reqest = new PositionVoltage(0);
    voltageOut = new VoltageOut(0);
    new PositionDutyCycle(0);

    pivotMotorLeft.setPosition(0);
    pivotMotorRight.setPosition(0);

    SmartDashboard.putNumber("Pivot Angle Value", 46);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // 0.545533063638327 High limit before 2048 mulitplier = 1,117.251714331294
    // 0.201015130025378 Low limit before 2048 multiplier = 411.6789862919741
    // absPos = absoluteEncoder.getPosition();
    // SmartDashboard.putNumber("Absolute Encoder Position", getAbsoluteEncoder());
    if(BasePIDGlobal.TEST_MODE == true) {
      SmartDashboard.putNumber("Pivot Left Position", pivotMotorLeft.getPosition().getValue());
      SmartDashboard.putNumber("Pivot Right Position", pivotMotorRight.getPosition().getValue());
      SmartDashboard.putBoolean("Pivot SoftLimit", getSoftLimit());
      // SmartDashboard.putBoolean("limit", limit);
    }

    if (absPos == PivotGlobalValues.PIVOT_NEUTRAL_ANGLE) {
      GlobalsValues.PivotGlobalValues.IS_NEUTRAL = true;
    }
  }

  /**
   * Stops the pivot motors
   *
   * @return void
   */
  public void stopMotors() {
    pivotMotorLeft.stopMotor();
    pivotMotorRight.stopMotor();
    voltageOut.Output = -0.014;
    pivotMotorLeft.setControl(voltageOut);
    pivotMotorRight.setControl(voltageOut);
  }

  /**
   * Set the position of the left and right pivot motors
   *
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
   *
   * @return double, the position of the pivot motor
   */
  public double getPivotLeftPos() {
    return pivotMotorLeft.getPosition().getValue();
  }

  public double getPivotRightPos() {
    return pivotMotorRight.getPosition().getValue();
  }

  public double getPivotPos() {
    return (getPivotLeftPos() + getPivotRightPos()) / 2;
  }

  /**
   * Run distance through a best fit line and return the value
   *
   * @param distance
   * @return double, the position of the pivot motor
   */
  public double shootPos(double distance) {
    // line function
    // TODO: do stuf
    return 0.0;
  }

  public void resetEncoders() {
    pivotMotorLeft.setPosition(0);
    pivotMotorRight.setPosition(0);
  }

  public void toggleSoftStop() {
    PivotGlobalValues.soft_limit_enabled = !PivotGlobalValues.soft_limit_enabled;
    leftSoftLimitConfig.ReverseSoftLimitEnable = PivotGlobalValues.soft_limit_enabled;
    // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    leftSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    // rightSoftLimitConfig.ForwardSoftLimitEnable = PivotGlobalValues.soft_limit_enabled;
    rightSoftLimitConfig.ReverseSoftLimitEnable = PivotGlobalValues.soft_limit_enabled;
    // rightSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    pivotMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    pivotMotorRight.getConfigurator().apply(rightSoftLimitConfig);
  }

  /**
   * // * Get the absolute encoder position // * // * @return double, the absolute encoder position
   * of the pivot motor //
   */
  // public double getAbsoluteEncoder() {
  //   // return actualAbsEnc.getAbsolutePosition() * 2048;
  //   if (absoluteEncoder.getPosition() > 190) {
  //     return 0;
  //   }
  //   else {
  //     return absoluteEncoder.getPosition();
  //   }
  // }

  public void movePivot(double velocity) {
    if (Math.abs(velocity) >= deadband) {
      pivotMotorLeft.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));
      pivotMotorRight.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));

      SmartDashboard.putNumber("PivotLeft Velo Error", pivotMotorLeft.get() - velocity);
      SmartDashboard.putNumber("PivotRight Velo Error", pivotMotorLeft.get() - velocity);
    } else {
      stopMotors();
    }
  }

  public void setPivot(double pos) {
    pivotMotorLeft.setControl(vel_voltage.withVelocity(pos));
    pivotMotorRight.setControl(vel_voltage.withVelocity(pos));
  }

  public void toggleLimit() {
    PivotGlobalValues.is_SOFTLIMIT = !PivotGlobalValues.is_SOFTLIMIT;
  }

  // public void recalibrateEncoders() {
  //   PivotGlobalValues.offset = PivotGlobalValues.PIVOT_NEUTRAL_ANGLE - getAbsoluteEncoder();
  // }

  public double getPivotPositionAvg() {
    return (getPivotLeftPos() + getPivotRightPos()) / 2;
  }

  public boolean getSoftLimit() {
    return PivotGlobalValues.is_SOFTLIMIT;
  }

  // public void CalibratePivot() {
  // limit = !limit;
  // if (limit) {
  // leftSoftLimitConfig.ForwardSoftLimitEnable = true;
  // leftSoftLimitConfig.ReverseSoftLimitEnable = true;
  // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
  // leftSoftLimitConfig.ReverseSoftLimitThreshold = 415;

  // rightSoftLimitConfig.ForwardSoftLimitEnable = true;
  // rightSoftLimitConfig.ReverseSoftLimitEnable = true;
  // rightSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
  // rightSoftLimitConfig.ReverseSoftLimitThreshold = 415;
  // }

  // else {
  // leftSoftLimitConfig.ForwardSoftLimitEnable = false;
  // leftSoftLimitConfig.ReverseSoftLimitEnable = false;

  // rightSoftLimitConfig.ForwardSoftLimitEnable = false;
  // rightSoftLimitConfig.ReverseSoftLimitEnable = false;
  // }

  // // pivotLeftConfiguration.SoftwareLimitSwitch = leftSoftLimitConfig;
  // // pivotRightConfiguration.SoftwareLimitSwitch = rightSoftLimitConfig;

  // pivotMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
  // pivotMotorRight.getConfigurator().apply(rightSoftLimitConfig);
  // }
}
