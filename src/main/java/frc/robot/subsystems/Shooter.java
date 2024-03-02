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
import frc.robot.utils.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX leftFalcon;
  private TalonFX rightFalcon;
  private TalonFX kraken;

  private TalonFXConfigurator leftShootConfigurator;
  private TalonFXConfigurator rightShootConfigurator;
  private TalonFXConfigurator krakenConfigurator;

  private Slot0Configs leftShootConfigs;
  private Slot0Configs rightShootConfigs;
  private Slot0Configs krakenConfigs;

  private MotorOutputConfigs shooterConfigs;

  private CurrentLimitsConfigs leftShootCurrentConfig;
  private CurrentLimitsConfigs rightShootCurrentConfig;
  private CurrentLimitsConfigs krakenCurrentConfig;

  private ClosedLoopRampsConfigs leftShootRampConfig;
  private ClosedLoopRampsConfigs rightShootRampConfig;
  private ClosedLoopRampsConfigs krakenRampConfig;

  private VelocityVoltage m_request;

  private boolean toggleShooter;

  public Shooter() {
    leftFalcon = new TalonFX(ShooterConstants.FALCON_LEFT_ID);
    rightFalcon = new TalonFX(ShooterConstants.FALCON_RIGHT_ID);
    kraken = new TalonFX(ShooterConstants.KRAKEN_ID);

    shooterConfigs = new MotorOutputConfigs();

    leftShootConfigurator = leftFalcon.getConfigurator();
    rightShootConfigurator = rightFalcon.getConfigurator();
    krakenConfigurator = kraken.getConfigurator();

    leftShootConfigs = new Slot0Configs();
    rightShootConfigs = new Slot0Configs();
    krakenConfigs = new Slot0Configs();

    leftFalcon.getConfigurator().apply(new TalonFXConfiguration());
    rightFalcon.getConfigurator().apply(new TalonFXConfiguration());
    kraken.getConfigurator().apply(new TalonFXConfiguration());

    shooterConfigs.NeutralMode = NeutralModeValue.Brake;
    leftShootConfigurator.apply(shooterConfigs);
    rightShootConfigurator.apply(shooterConfigs);
    krakenConfigurator.apply(shooterConfigs);

    leftShootConfigs.kP = ShooterConstants.SHOOTER_PID_LEFT_P;
    leftShootConfigs.kI = ShooterConstants.SHOOTER_PID_LEFT_I;
    leftShootConfigs.kD = ShooterConstants.SHOOTER_PID_LEFT_D;

    rightShootConfigs.kP = ShooterConstants.SHOOTER_PID_RIGHT_P;
    rightShootConfigs.kI = ShooterConstants.SHOOTER_PID_RIGHT_I;
    rightShootConfigs.kD = ShooterConstants.SHOOTER_PID_RIGHT_D;

    krakenConfigs.kP = ShooterConstants.KRAKEN_P;
    krakenConfigs.kI = ShooterConstants.KRAKEN_I;
    krakenConfigs.kD = ShooterConstants.KRAKEN_D;

    leftFalcon.getConfigurator().apply(leftShootConfigs);
    rightFalcon.getConfigurator().apply(rightShootConfigs);
    kraken.getConfigurator().apply(krakenConfigs);

    leftShootCurrentConfig = new CurrentLimitsConfigs();
    rightShootCurrentConfig = new CurrentLimitsConfigs();
    krakenCurrentConfig = new CurrentLimitsConfigs();

    leftShootRampConfig = new ClosedLoopRampsConfigs();
    rightShootRampConfig = new ClosedLoopRampsConfigs();
    krakenRampConfig = new ClosedLoopRampsConfigs();

    leftShootCurrentConfig.SupplyCurrentLimit = 100;
    leftShootCurrentConfig.StatorCurrentLimit = 100;

    rightShootCurrentConfig.SupplyCurrentLimit = 100;
    rightShootCurrentConfig.StatorCurrentLimit = 100;

    krakenCurrentConfig.SupplyCurrentLimit = 100;
    krakenCurrentConfig.StatorCurrentLimit = 100;

    leftFalcon.getConfigurator().apply(leftShootCurrentConfig);
    rightFalcon.getConfigurator().apply(rightShootCurrentConfig);
    kraken.getConfigurator().apply(krakenCurrentConfig);

    leftShootRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    rightShootRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    krakenRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    leftFalcon.getConfigurator().apply(leftShootRampConfig);
    rightFalcon.getConfigurator().apply(rightShootRampConfig);
    kraken.getConfigurator().apply(krakenRampConfig);

    toggleShooter = false;
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {}

  /**
   * Sets the velocity of the shooter motors
   * @param left Left motor speed in RPM
   * @param right Right motor speed in RPM
   * @return void
   */
  public void setShooterVelocity(double left, double right) {
    leftFalcon.setControl(m_request.withVelocity(left));
    rightFalcon.setControl(m_request.withVelocity(right));
  }

  /**
  * Operator command to toggle shooter using Left Trigger
  * @param None
  * @return void
  */
  public void toggleShooterVelocity() {
    toggleShooter = !toggleShooter;
    if (toggleShooter) {
      setShooterVelocity(ShooterConstants.SHOOTER_SPEED, -ShooterConstants.SHOOTER_SPEED);
    } else {
      stopShooter();
    }
  }

  /**
  * Gets the RPM of the left falcon motor on the shooter
  * @param None
  * @return double, leftFalcon RPM
  */
  public double getLeftShooterVelocity() {
    return leftFalcon.getRotorVelocity().getValue();
  }

  /**
  * Gets the RP of the right falcon motor on the shooter
  * @param None
  * @return double, rightFalcon RPM
  */
  public double getRightShooterVelocity() {
    return leftFalcon.getRotorVelocity().getValue();
  }

  /**
   * Sets the velocity of the kraken motor
   * @param speed in RPM
   * @return void
   */
  public void setKrakenVelocity(double speed) {
    kraken.setControl(m_request.withVelocity(speed));
  }

  /**
   * Gets the velocity of the kraken motor
   * @param None
   * @return double, kraken RPM
   */
  public double getKrakenVelocity() {
    return kraken.getRotorVelocity().getValue();
  }

  /**
   * Stops the shooter motors
   * @param None
   * @return void
   */
  public void stopShooter() {
    leftFalcon.stopMotor();
    rightFalcon.stopMotor();
  }

  /**
   * Stops the kraken motor
   * @param None
   * @return void
   */
  public void stopKraken() {
    kraken.stopMotor();
  }

  /**
   * Stops all motors
   * @param None
   * @return void
   */
  public void stopAllMotors() {
    stopShooter();
    stopKraken();
  }
}
