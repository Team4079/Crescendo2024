// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.Position;

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

import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.PivotConstants;
import frc.robot.utils.Constants.ShooterConstants;

@SuppressWarnings("unused")
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX leftFalcon;
  private TalonFX rightFalcon;
  private TalonFX karen;

  private TalonFXConfigurator leftShootConfigurator;
  private TalonFXConfigurator rightShootConfigurator;
  private TalonFXConfigurator karenConfigurator;

  private Slot0Configs leftShootConfigs;
  private Slot0Configs rightShootConfigs;
  private Slot0Configs karenConfigs;

  private MotorOutputConfigs shooterConfigs;

  private CurrentLimitsConfigs leftShootCurrentConfig;
  private CurrentLimitsConfigs rightShootCurrentConfig;
  private CurrentLimitsConfigs karenCurrentConfig;

  private ClosedLoopRampsConfigs leftShootRampConfig;
  private ClosedLoopRampsConfigs rightShootRampConfig;
  private ClosedLoopRampsConfigs karenRampConfig;

  private VelocityVoltage m_request;

  private boolean toggleShooter;

  public Shooter() {
    leftFalcon = new TalonFX(ShooterConstants.FALCON_LEFT_ID);
    rightFalcon = new TalonFX(ShooterConstants.FALCON_RIGHT_ID);
    karen = new TalonFX(ShooterConstants.KAREN_ID);

    shooterConfigs = new MotorOutputConfigs();

    leftShootConfigurator = leftFalcon.getConfigurator();
    rightShootConfigurator = rightFalcon.getConfigurator();
    karenConfigurator = karen.getConfigurator();

    leftShootConfigs = new Slot0Configs();
    rightShootConfigs = new Slot0Configs();
    karenConfigs = new Slot0Configs();

    leftFalcon.getConfigurator().apply(new TalonFXConfiguration());
    rightFalcon.getConfigurator().apply(new TalonFXConfiguration());
    karen.getConfigurator().apply(new TalonFXConfiguration());

    shooterConfigs.NeutralMode = NeutralModeValue.Brake;
    leftShootConfigurator.apply(shooterConfigs);
    rightShootConfigurator.apply(shooterConfigs);
    karenConfigurator.apply(shooterConfigs);

    leftShootConfigs.kP = ShooterConstants.SHOOTER_PID_LEFT_P;
    leftShootConfigs.kI = ShooterConstants.SHOOTER_PID_LEFT_I;
    leftShootConfigs.kD = ShooterConstants.SHOOTER_PID_LEFT_D;

    rightShootConfigs.kP = ShooterConstants.SHOOTER_PID_RIGHT_P;
    rightShootConfigs.kI = ShooterConstants.SHOOTER_PID_RIGHT_I;
    rightShootConfigs.kD = ShooterConstants.SHOOTER_PID_RIGHT_D;

    karenConfigs.kP = ShooterConstants.KAREN_P;
    karenConfigs.kI = ShooterConstants.KAREN_I;
    karenConfigs.kD = ShooterConstants.KAREN_D;

    leftFalcon.getConfigurator().apply(leftShootConfigs);
    rightFalcon.getConfigurator().apply(rightShootConfigs);
    karen.getConfigurator().apply(karenConfigs);

    leftShootCurrentConfig = new CurrentLimitsConfigs();
    rightShootCurrentConfig = new CurrentLimitsConfigs();
    karenCurrentConfig = new CurrentLimitsConfigs();

    leftShootRampConfig = new ClosedLoopRampsConfigs();
    rightShootRampConfig = new ClosedLoopRampsConfigs();
    karenRampConfig = new ClosedLoopRampsConfigs();

    leftShootCurrentConfig.SupplyCurrentLimit = 100;
    leftShootCurrentConfig.StatorCurrentLimit = 100;

    rightShootCurrentConfig.SupplyCurrentLimit = 100;
    rightShootCurrentConfig.StatorCurrentLimit = 100;

    karenCurrentConfig.SupplyCurrentLimit = 100;
    karenCurrentConfig.StatorCurrentLimit = 100;

    leftFalcon.getConfigurator().apply(leftShootCurrentConfig);
    rightFalcon.getConfigurator().apply(rightShootCurrentConfig);
    karen.getConfigurator().apply(karenCurrentConfig);

    leftShootRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    rightShootRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    karenRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    leftFalcon.getConfigurator().apply(leftShootRampConfig);
    rightFalcon.getConfigurator().apply(rightShootRampConfig);
    karen.getConfigurator().apply(karenRampConfig);

    toggleShooter = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setShooterVelocity(double left, double right) {
    leftFalcon.setControl(m_request.withVelocity(left));
    rightFalcon.setControl(m_request.withVelocity(right));
  }

  public void toggleShooterVelocity() {
    toggleShooter = !toggleShooter;
    if (toggleShooter) {
      setShooterVelocity(ShooterConstants.SHOOTER_SPEED, -ShooterConstants.SHOOTER_SPEED);
    } else {
      stopShooter();
    }
  }

  public double getLeftShooterVelocity() {
    return leftFalcon.getRotorVelocity().getValue();
  }

  public double getRightShooterVelocity() {
    return leftFalcon.getRotorVelocity().getValue();
  }

  public void setKarenVelocity(double speed) {
    karen.setControl(m_request.withVelocity(speed));
  }

  public double getKarenVelocity() {
    return karen.getRotorVelocity().getValue();
  }

  public void stopShooter() {
    leftFalcon.stopMotor();
    rightFalcon.stopMotor();
  }

  public void stopKaren() {
    karen.stopMotor();
  }

  public void stopAllMotors() {
    stopShooter();
    stopKaren();
  }
}
