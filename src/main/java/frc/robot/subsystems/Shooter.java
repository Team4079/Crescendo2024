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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/**
 * The {@link Shooter} class includes all the motors to shoot the power cells.
 */
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX leftFalcon;
  private TalonFX rightFalcon;
  private TalonFX passthroughKraken;

  private TalonFXConfigurator leftShootConfigurator;
  private TalonFXConfigurator rightShootConfigurator;
  private TalonFXConfigurator passthroughKrakenConfigurator;

  private Slot0Configs leftShootConfigs;
  private Slot0Configs rightShootConfigs;
  private Slot0Configs passthroughKrakenConfigs;

  private MotorOutputConfigs shooterConfigs;

  private CurrentLimitsConfigs leftShootCurrentConfig;
  private CurrentLimitsConfigs rightShootCurrentConfig;
  private CurrentLimitsConfigs passthroughKrakenCurrentConfig;

  private ClosedLoopRampsConfigs leftShootRampConfig;
  private ClosedLoopRampsConfigs rightShootRampConfig;
  private ClosedLoopRampsConfigs passthroughKrakenRampConfig;

  private VelocityVoltage velocitySetPoint;

  private boolean toggleShooter;

  private DigitalInput ringSensor;

  private boolean leftFalconIsStopped;
  private boolean rightFalconIsStopped;
  private boolean passthroughKrakenIsStopped;

  private Timer timer;

  public Shooter() {
    leftFalcon = new TalonFX(ShooterGlobalValues.FALCON_LEFT_ID);
    rightFalcon = new TalonFX(ShooterGlobalValues.FALCON_RIGHT_ID);
    passthroughKraken = new TalonFX(ShooterGlobalValues.KRAKEN_ID);

    ringSensor = new DigitalInput(ShooterGlobalValues.RING_SENSOR_PORT);

    shooterConfigs = new MotorOutputConfigs();

    leftShootConfigurator = leftFalcon.getConfigurator();
    rightShootConfigurator = rightFalcon.getConfigurator();
    passthroughKrakenConfigurator = passthroughKraken.getConfigurator();

    leftShootConfigs = new Slot0Configs();
    rightShootConfigs = new Slot0Configs();
    passthroughKrakenConfigs = new Slot0Configs();

    leftFalcon.getConfigurator().apply(new TalonFXConfiguration());
    rightFalcon.getConfigurator().apply(new TalonFXConfiguration());
    passthroughKraken.getConfigurator().apply(new TalonFXConfiguration());

    shooterConfigs.NeutralMode = NeutralModeValue.Brake;
    leftShootConfigurator.apply(shooterConfigs);
    rightShootConfigurator.apply(shooterConfigs);
    passthroughKrakenConfigurator.apply(shooterConfigs);

    leftShootConfigs.kP = ShooterGlobalValues.SHOOTER_PID_LEFT_P;
    leftShootConfigs.kI = ShooterGlobalValues.SHOOTER_PID_LEFT_I;
    leftShootConfigs.kD = ShooterGlobalValues.SHOOTER_PID_LEFT_D;
    leftShootConfigs.kV = ShooterGlobalValues.SHOOTER_PID_LEFT_V;

    rightShootConfigs.kP = ShooterGlobalValues.SHOOTER_PID_RIGHT_P;
    rightShootConfigs.kI = ShooterGlobalValues.SHOOTER_PID_RIGHT_I;
    rightShootConfigs.kD = ShooterGlobalValues.SHOOTER_PID_RIGHT_D;
    rightShootConfigs.kV = ShooterGlobalValues.SHOOTER_PID_RIGHT_V;

    passthroughKrakenConfigs.kP = ShooterGlobalValues.KRAKEN_P;
    passthroughKrakenConfigs.kI = ShooterGlobalValues.KRAKEN_I;
    passthroughKrakenConfigs.kD = ShooterGlobalValues.KRAKEN_D;
    passthroughKrakenConfigs.kV = ShooterGlobalValues.KRAKEN_V;

    leftFalcon.getConfigurator().apply(leftShootConfigs);
    rightFalcon.getConfigurator().apply(rightShootConfigs);
    passthroughKraken.getConfigurator().apply(passthroughKrakenConfigs);

    leftShootCurrentConfig = new CurrentLimitsConfigs();
    rightShootCurrentConfig = new CurrentLimitsConfigs();
    passthroughKrakenCurrentConfig = new CurrentLimitsConfigs();

    leftShootRampConfig = new ClosedLoopRampsConfigs();
    rightShootRampConfig = new ClosedLoopRampsConfigs();
    passthroughKrakenRampConfig = new ClosedLoopRampsConfigs();

    leftShootCurrentConfig.SupplyCurrentLimit = 100;
    leftShootCurrentConfig.StatorCurrentLimit = 80;
    leftShootCurrentConfig.StatorCurrentLimitEnable = true;

    rightShootCurrentConfig.SupplyCurrentLimit = 100;
    rightShootCurrentConfig.StatorCurrentLimit = 80;
    rightShootCurrentConfig.StatorCurrentLimitEnable = true;

    passthroughKrakenCurrentConfig.SupplyCurrentLimit = 100;
    passthroughKrakenCurrentConfig.StatorCurrentLimit = 80;
    passthroughKrakenCurrentConfig.StatorCurrentLimitEnable = true;

    leftFalcon.getConfigurator().apply(leftShootCurrentConfig);
    rightFalcon.getConfigurator().apply(rightShootCurrentConfig);
    passthroughKraken.getConfigurator().apply(passthroughKrakenCurrentConfig);

    leftShootRampConfig.DutyCycleClosedLoopRampPeriod = 0;
    rightShootRampConfig.DutyCycleClosedLoopRampPeriod = 0;
    passthroughKrakenRampConfig.DutyCycleClosedLoopRampPeriod = 0.5;

    leftFalcon.getConfigurator().apply(leftShootRampConfig);
    rightFalcon.getConfigurator().apply(rightShootRampConfig);
    passthroughKraken.getConfigurator().apply(passthroughKrakenRampConfig);

    toggleShooter = false;

    velocitySetPoint = new VelocityVoltage(0);

    timer = new Timer();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Shooter", leftFalcon.getRotorVelocity().getValue());
    SmartDashboard.putNumber("Right Shooter Velocity", rightFalcon.getRotorVelocity().getValue());
    SmartDashboard.putNumber("Left Shooter Error", leftFalcon.getVelocity().getValue() - 50);
    SmartDashboard.putNumber("Right Shooter Error", rightFalcon.getVelocity().getValue() - 50);
    SmartDashboard.putNumber("Kraken Velocity", passthroughKraken.getRotorVelocity().getValue());

    if (getRingSensor()) {
      timer.start();
      if (timer.get() < 0.35) {
        ShooterGlobalValues.HAS_PIECE = true;
      }
    } else {
      ShooterGlobalValues.HAS_PIECE = false;
      timer.reset();
    }

    SmartDashboard.putBoolean("has peice", ShooterGlobalValues.HAS_PIECE);
    // setKrakenVelocity(-25);
    // setShooterVelocity(-70, -70);
  }

  /**
   * Sets the velocity of the shooter motors
   * 
   * @param left  Left motor speed in RPS
   * @param right Right motor speed in RPS
   * @return void
   */
  public void setShooterVelocity(double left, double right) {
    leftFalcon.setControl(velocitySetPoint.withVelocity(left));
    rightFalcon.setControl(velocitySetPoint.withVelocity(right));
    leftFalconIsStopped = false;
    rightFalconIsStopped = false;
  }

  /**
   * Operator command to toggle shooter using Left Trigger
   * 
   * @param void
   * @return void
   */
  public void toggleShooterVelocity() {
    toggleShooter = !toggleShooter;
    if (toggleShooter) {
      setShooterVelocity(ShooterGlobalValues.SHOOTER_SPEED, -ShooterGlobalValues.SHOOTER_SPEED);
    } else {
      stopShooter();
    }
  }

  /**
   * Gets the RPM of the left falcon motor on the shooter
   * 
   * @param void
   * @return double, leftFalcon RPM
   */
  public double getLeftShooterVelocity() {
    return leftFalcon.getRotorVelocity().getValue();
  }

  /**
   * Gets the RP of the right falcon motor on the shooter
   * 
   * @param void
   * @return double, rightFalcon RPS
   */
  public double getRightShooterVelocity() {
    return rightFalcon.getRotorVelocity().getValue();
  }

  /**
   * Sets the velocity of the kraken motor
   * 
   * @param speed in RPM
   * @return void
   */
  public void setKrakenVelocity(double speed) {
    passthroughKraken.setControl(velocitySetPoint.withVelocity(speed));
    passthroughKrakenIsStopped = false;
  }

  /**
   * Gets the velocity of the kraken motor
   * 
   * @param void
   * @return double, kraken RPM
   */
  public double getKrakenVelocity() {
    return passthroughKraken.getRotorVelocity().getValue();
  }

  /**
   * Stops the shooter motors
   * 
   * @param void
   * @return void
   */
  public void stopShooter() {
    if (!leftFalconIsStopped || !rightFalconIsStopped) {
      leftFalcon.stopMotor();
      rightFalcon.stopMotor();
      leftFalconIsStopped = true;
      rightFalconIsStopped = true;
    }
  }

  /**
   * Stops the kraken motor
   * 
   * @param void
   * @return void
   */
  public void stopKraken() {
    if (!passthroughKrakenIsStopped) {
      passthroughKraken.stopMotor();
      passthroughKrakenIsStopped = true;
    }
  }

  /**
   * Stops all motors
   * 
   * @param void
   * @return void
   */
  public void stopAllMotors() {
    stopShooter();
    stopKraken();
  }

  public void stopAllMotorsAuto() {
    passthroughKraken.stopMotor();
    stopShooter();
  }

  /**
   * Gets the value of the ring sensor
   * 
   * @return boolean, ring sensor value, default false
   */
  public boolean getRingSensor() {
    SmartDashboard.putBoolean("sensor boolean", !ringSensor.get());
    return !ringSensor.get();
  }
}