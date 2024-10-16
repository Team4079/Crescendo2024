// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues;

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

  /**
   * Creates a new SetPivot command.
   *
   * @param pivot The Pivot subsystem used by this command.
   * @param pos The target position for the pivot.
   */
  public SetPivot(Pivot pivot, double pos) {
    this.pivot = pivot;
    this.pos = pos;
    pidController =
        new PIDController(
            GlobalsValues.PivotGlobalValues.SETPIVOT_PID_P,
            GlobalsValues.PivotGlobalValues.SETPIVOT_PID_I,
            GlobalsValues.PivotGlobalValues.SETPIVOT_PID_D);
    addRequirements(pivot);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    deadband = 0.5;
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    double velocity = pidController.calculate(pivot.getPivotPos(), pos);
    motorPivot(velocity, pivot, pos, deadband);
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
    SmartDashboard.putNumber("Pivot Error", pos - pivot.getPivotPos());
    SmartDashboard.putNumber("Target Pivot Pos", pos);
    SmartDashboard.putNumber("Pivot Velocity", velocity);

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
    return Math.abs(pivot.getPivotPos() - pos) <= deadband;
  }
}
