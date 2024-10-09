// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

public class StagePassPivot extends Command {
  private Pivot pivot;
  private Timer timer;
  private double deadband;
  private boolean isDone;
  private double pos;
  private double velocity;
  private double rps;

  // Horizontal PID and offset
  private double horizontalError;

  // Rotation PID and offset
  private PIDController rotationalController;
  private PIDController velocityPIDController;

  private double timeout = 0;
  private boolean end = false;

  /** Creates a new PivotShooterSetUp. */
  public StagePassPivot(Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    timer = new Timer();
    rotationalController =
        new PIDController(
            BasePIDGlobal.ROTATIONAL_PID.p,
            BasePIDGlobal.ROTATIONAL_PID.i,
            BasePIDGlobal.ROTATIONAL_PID.d);
    velocityPIDController = new PIDController(0.00825, 0.000000, 0.00035);
    rotationalController.setTolerance(3);
    this.pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    deadband = 0.1;
    isDone = false;
    pos = 15;

    // shooter.setShooterVelocity(-rps, -rps);\[]

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = velocityPIDController.calculate(pivot.getPivotPos(), pos);
    SmartDashboard.putNumber("Error Pivot Right", -pivot.getPivotPos() + pos);
    SmartDashboard.putNumber("Error Pivot Left", -pivot.getPivotPos() + pos);

    if (Math.abs(pivot.getPivotPos() - pos) < deadband) {
      pivot.stopMotors();
    } else {
      pivot.movePivot(velocity);
    }

    if (Math.abs(pivot.getPivotPos() - pos) <= deadband) {
      timer.start();
      if (timer.get() >= 0.1) {
        isDone = true;
      }
    } else {
      timer.reset();
      isDone = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
