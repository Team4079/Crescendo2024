// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Hell.IntakeConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private TalonFX intakeKraken;
  private TalonFXConfigurator intakeKrakenConfigurator;
  private Slot0Configs slot0Config;
  private DutyCycleOut m_request;
  
  public Intake() {
    this.intakeKraken = new TalonFX(IntakeConstants.INTAKE_MOTOR_TOP_ID);

    intakeKrakenConfigurator = intakeKraken.getConfigurator();

    intakeKraken.getConfigurator().apply(new TalonFXConfiguration());

    m_request = new DutyCycleOut(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotors() {
    intakeKraken.stopMotor();
  }

  public void setVelocity(double velocity) {
    intakeKraken.setControl(m_request.withOutput(velocity));
  }
}
