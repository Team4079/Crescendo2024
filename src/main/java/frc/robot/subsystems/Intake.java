// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues.IntakeGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

/**
 * The Intake subsystem controls the intake mechanism of the robot. It uses a TalonFX motor
 * controller to manage the intake motor.
 */
public class Intake extends SubsystemBase {
  private final TalonFX intakeKaren;
  private final VelocityVoltage request;
  private boolean intakeIsStopped;

  /**
   * Constructs a new Intake subsystem. Initializes the TalonFX motor controller and configures its
   * settings.
   */
  public Intake() {
    this.intakeKaren = new TalonFX(IntakeGlobalValues.INTAKE_MOTOR_ID);

    TalonFXConfigurator intakeKarenConfigurator = intakeKaren.getConfigurator();

    Slot0Configs karenConfig = new Slot0Configs();

    intakeKaren.getConfigurator().apply(new TalonFXConfiguration());

    MotorOutputConfigs intakeConfigs = new MotorOutputConfigs();

    intakeKarenConfigurator.apply(intakeConfigs);

    karenConfig.kV = IntakeGlobalValues.INTAKE_PID_V;
    karenConfig.kP = IntakeGlobalValues.INTAKE_PID_P;
    karenConfig.kI = IntakeGlobalValues.INTAKE_PID_I;
    karenConfig.kD = IntakeGlobalValues.INTAKE_PID_D;

    intakeKaren.getConfigurator().apply(karenConfig);

    CurrentLimitsConfigs karenCurrentConfig = new CurrentLimitsConfigs();

    ClosedLoopRampsConfigs karenRampConfig = new ClosedLoopRampsConfigs();

    karenCurrentConfig.SupplyCurrentLimit = 100;
    karenCurrentConfig.StatorCurrentLimit = 100;

    intakeKaren.getConfigurator().apply(karenCurrentConfig);

    karenRampConfig.DutyCycleClosedLoopRampPeriod = 0.01;

    intakeKaren.getConfigurator().apply(karenRampConfig);

    request = new VelocityVoltage(0);
  }

  /**
   * This method will be called once per scheduler run. Updates the SmartDashboard with the current
   * intake motor velocity.
   */
  @Override
  public void periodic() {
    if(BasePIDGlobal.TEST_MODE == true) {
      SmartDashboard.putNumber("Intake Velocity", intakeKaren.getRotorPosition().getValue());
    }
  }

  /**
   * Sets the intake motor to a specific speed.
   *
   * @param speed The speed to set the intake motor to.
   */
  public void setIntakeVelocity(double speed) {
    intakeKaren.setControl(request.withVelocity(speed));
    intakeIsStopped = false;
  }

  /**
   * Returns the current status of the intake.
   *
   * @return true if the intake is running, false if it is stopped.
   */
  public boolean getIntakeStatus() {
    return !intakeIsStopped;
  }

  /** Stops the intake motor. */
  public void stopKraken() {
    if (!intakeIsStopped) {
      intakeKaren.stopMotor();
      intakeIsStopped = true;
    }
  }
}
