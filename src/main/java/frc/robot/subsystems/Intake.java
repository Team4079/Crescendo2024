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
// import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;

/**
 * The {@link Intake} class includes all the motors to intake notes.
 * 
 *
 * 
 */
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private TalonFX intakeKaren;
  private TalonFXConfigurator intakeKarenConfigurator;
  private Slot0Configs karenConfig;

  private MotorOutputConfigs intakeConfigs;

  private CurrentLimitsConfigs karenCurrentConfig;

  private ClosedLoopRampsConfigs karenRampConfig;

  private VelocityVoltage m_request;
  // private VoltageOut m_out;

  public Intake() {
    this.intakeKaren = new TalonFX(IntakeConstants.INTAKE_MOTOR_ID);

    intakeKarenConfigurator = intakeKaren.getConfigurator();

    karenConfig = new Slot0Configs();

    intakeKaren.getConfigurator().apply(new TalonFXConfiguration());

    intakeConfigs = new MotorOutputConfigs();

    intakeKarenConfigurator.apply(intakeConfigs);

    karenConfig.kV = IntakeConstants.INTAKE_PID_V;
    karenConfig.kP = IntakeConstants.INTAKE_PID_P;
    karenConfig.kI = IntakeConstants.INTAKE_PID_I;
    karenConfig.kP = IntakeConstants.INTAKE_PID_D;

    intakeKaren.getConfigurator().apply(karenConfig);

    karenCurrentConfig = new CurrentLimitsConfigs();

    karenRampConfig = new ClosedLoopRampsConfigs();

    karenCurrentConfig.SupplyCurrentLimit = 100;
    karenCurrentConfig.StatorCurrentLimit = 100;

    intakeKaren.getConfigurator().apply(karenCurrentConfig);

    karenRampConfig.DutyCycleClosedLoopRampPeriod = 0.01;

    intakeKaren.getConfigurator().apply(karenRampConfig);

    m_request = new VelocityVoltage(0);
    // m_out = new VoltageOut(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Velocity", intakeKaren.getRotorPosition().getValue());
  }

  /**
   * Sets the intake motor to a specific speed
   * @param speed The speed to set the intake motor to
   * @return void
   */
  public void setIntakeVelocity(double speed) {
    intakeKaren.setControl(m_request.withVelocity(speed));
  }

  /**
   * Stops the intake motor
   * @param None
   * @return void
   */
  public void stopKraken() {
    intakeKaren.stopMotor();
  }
}
