// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.PivotConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;


public class Pivot extends SubsystemBase {
  /** Creates a new Intake. */

  private CANSparkMax pivotMotorTop;
  private CANSparkMax pivotMotorBottom;
  
  private SparkPIDController pivotPIDTop;
  private SparkPIDController pivotPIDBottom;

  public Pivot() {
    this.pivotMotorTop = new CANSparkMax(PivotConstants.PIVOT_MOTOR_TOP_ID, CANSparkMax.MotorType.kBrushless);
    this.pivotMotorTop = new CANSparkMax(PivotConstants.PIVOT_MOTOR_BOTTOM_ID, CANSparkMax.MotorType.kBrushless);

    this.pivotPIDTop = pivotMotorTop.getPIDController();
    this.pivotPIDBottom = pivotMotorBottom.getPIDController();

    this.pivotPIDTop.setP(PivotConstants.PIVOT_PID_TOP_P);
    this.pivotPIDTop.setI(PivotConstants.PIVOT_PID_TOP_I);
    this.pivotPIDTop.setD(PivotConstants.PIVOT_PID_TOP_D);

    this.pivotPIDBottom.setP(PivotConstants.PIVOT_PID_BOTTOM_P);
    this.pivotPIDBottom.setI(PivotConstants.PIVOT_PID_BOTTOM_I);
    this.pivotPIDBottom.setD(PivotConstants.PIVOT_PID_BOTTOM_D);

    pivotMotorTop.setInverted(PivotConstants.isInverted);
    pivotMotorBottom.setInverted(!PivotConstants.isInverted);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stopMotors() {
    pivotMotorTop.stopMotor();
    pivotMotorBottom.stopMotor();
  }

  public void setVelocity(double top, double bottom) {
    pivotPIDTop.setReference(top, CANSparkMax.ControlType.kVelocity);
    pivotPIDBottom.setReference(bottom, CANSparkMax.ControlType.kVelocity);
  }
}
