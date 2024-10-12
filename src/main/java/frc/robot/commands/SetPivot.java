// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;

/** The {@link SetPivot} class is a command that sets the pivot to a specified position. */
public class SetPivot extends Command {
  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  /** The target position for the pivot. */
  private final double pos;

  /** The PID controller for position adjustment. */
  private final PIDController pidController;

  /** The deadband value for the position error. */
  private double deadband;

  /** Indicates whether the command is done. */
  private boolean isDone;

  private static double currPos = 0.0;

  /**
   * Creates a new SetPivot command.
   *
   * @param pivot The Pivot subsystem used by this command.
   * @param pos The target position for the pivot.
   */
  public SetPivot(Pivot pivot, double pos) {
    this.pivot = pivot;
    this.pos = pos;
    pidController = new PIDController(0.00825, 0.000000, 0.00035);
    addRequirements(pivot);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    deadband = 0.5;
    isDone = false;
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    double velocity = pidController.calculate(pivot.getPivotPos(), pos);
    motorPivot(velocity, pivot, pos, deadband);

    if (Math.abs(pivot.getPivotPos() - pos) <= deadband) {
      isDone = true;
    }
  }

  /**
   * Updates the motor velocity and SmartDashboard values based on the pivot position.
   *
   * @param velocity The calculated velocity for the pivot.
   * @param pivot The Pivot subsystem used by this command.
   * @param pos The target position for the pivot.
   * @param deadband The deadband value for the position error.
   */
  public static void motorPivot(double velocity, Pivot pivot, double pos, double deadband) {
    SmartDashboard.putNumber("Error Pivot", -pivot.getPivotPos() + pos);
    SmartDashboard.putNumber("Setpoint", pos);
    SmartDashboard.putNumber("Velocity Pivot", velocity);

    if (Math.abs(pivot.getPivotPos() - pos) < deadband) {
      pivot.stopMotors();
    } else {
      pivot.movePivot(velocity);
    }
  }

  /**
   * Called once the command ends or is interrupted.
   *
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    pivot.stopMotors();
  }

  /**
   * Returns true when the command should end.
   *
   * @return true if the command is done, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
