// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Hell.PassthroughConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;


public class Passthrough extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkMax passthroughMotorTop;
  private CANSparkMax passthroughMotorBottom;
  
  private SparkPIDController passthroughPIDTop;
  private SparkPIDController passthroughPIDBottom;

  public Passthrough() {
    this.passthroughMotorTop = new CANSparkMax(PassthroughConstants.PASSTHROUGH_MOTOR_TOP_ID, CANSparkMax.MotorType.kBrushless);
    this.passthroughMotorBottom = new CANSparkMax(PassthroughConstants.PASSTHROUGH_MOTOR_BOTTOM_ID, CANSparkMax.MotorType.kBrushless);

    this.passthroughPIDTop = passthroughMotorTop.getPIDController();
    this.passthroughPIDBottom = passthroughMotorBottom.getPIDController();

    this.passthroughPIDTop.setP(PassthroughConstants.PASSTHROUGH_PID_TOP_P);
    this.passthroughPIDTop.setI(PassthroughConstants.PASSTHROUGH_PID_TOP_I);
    this.passthroughPIDTop.setD(PassthroughConstants.PASSTHROUGH_PID_TOP_D);

    this.passthroughPIDBottom.setP(PassthroughConstants.PASSTHROUGH_PID_BOTTOM_P);
    this.passthroughPIDBottom.setI(PassthroughConstants.PASSTHROUGH_PID_BOTTOM_I);
    this.passthroughPIDBottom.setD(PassthroughConstants.PASSTHROUGH_PID_BOTTOM_D);

    passthroughMotorTop.setInverted(PassthroughConstants.isInverted);
    passthroughMotorBottom.setInverted(!PassthroughConstants.isInverted);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotors() {
    passthroughMotorTop.stopMotor();
    passthroughMotorBottom.stopMotor();
  }

  public void setVelocity(double top, double bottom) {
    passthroughPIDTop.setReference(top, CANSparkMax.ControlType.kVelocity);
    passthroughPIDBottom.setReference(bottom, CANSparkMax.ControlType.kVelocity);
  }
}
