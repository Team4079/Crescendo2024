// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkMax intakeMotorTop;
  private CANSparkMax intakeMotorBottom;
  
  private SparkPIDController intakePIDTop;
  private SparkPIDController intakePIDBottom;

  public Intake() {
    this.intakeMotorTop = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_TOP_ID, CANSparkMax.MotorType.kBrushless);
    this.intakeMotorBottom = new CANSparkMax(IntakeConstants.INTAKE_MOTOR_BOTTOM_ID, CANSparkMax.MotorType.kBrushless);

    this.intakePIDTop = intakeMotorTop.getPIDController();
    this.intakePIDBottom = intakeMotorBottom.getPIDController();

    this.intakePIDTop.setP(IntakeConstants.INTAKE_PID_TOP_P);
    this.intakePIDTop.setI(IntakeConstants.INTAKE_PID_TOP_I);
    this.intakePIDTop.setD(IntakeConstants.INTAKE_PID_TOP_D);

    this.intakePIDBottom.setP(IntakeConstants.INTAKE_PID_BOTTOM_P);
    this.intakePIDBottom.setI(IntakeConstants.INTAKE_PID_BOTTOM_I);
    this.intakePIDBottom.setD(IntakeConstants.INTAKE_PID_BOTTOM_D);

    intakeMotorTop.setInverted(IntakeConstants.isInverted);
    intakeMotorBottom.setInverted(!IntakeConstants.isInverted);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotors() {
    intakeMotorTop.stopMotor();
    intakeMotorBottom.stopMotor();
  }

  public void setVelocity(double top, double bottom) {
    intakePIDTop.setReference(top, CANSparkMax.ControlType.kVelocity);
    intakePIDBottom.setReference(bottom, CANSparkMax.ControlType.kVelocity);
  }
}
