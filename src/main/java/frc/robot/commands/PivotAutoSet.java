// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

/** The {@link RePivotAutoSet} class is a command that resets the pivot to its neutral position. */
public class PivotAutoSet extends Command {

  private Pivot pivot;
  private double pos;
  private PIDController pidController;
  private double velocity;
  private Timer timer;
  private double deadband;
  private boolean isDone;
  private Limelight limelight;
  

  // Get distance when after we mount the limelight

  /** Creates a new Shoot. */
  public PivotAutoSet(Pivot pivot, Limelight limelight) {
    this.pivot = pivot;
    this.limelight = limelight;
    timer = new Timer();
    pidController = new PIDController(0.00825, 0.000000, 0.00035);
    addRequirements(pivot, limelight);
  }

  // // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    deadband = 0.1;
    isDone = false;
    pos = PivotGlobalValues.PIVOT_SUBWOOFER_ANGLE;
    pos = limelight.getPivotPosition();
    // pidController.setTolerance(50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = pidController.calculate(pivot.getAbsoluteEncoder(), pos);
    SmartDashboard.putNumber("Error Pivot", -pivot.getAbsoluteEncoder() + pos);
    SmartDashboard.putNumber("Setpoint", pos);
    SmartDashboard.putNumber("Velocity Pivot", velocity);

    if (Math.abs(pivot.getAbsoluteEncoder() - pos) < deadband)
    {
       pivot.stopMotors();
    }

    else {
      pivot.movePivot(velocity);
    }
    
    if (Math.abs(pivot.getAbsoluteEncoder() - pos) <= deadband)
    {
      timer.start();
      if (timer.get() >= 0.1) {
        isDone = true;
      }
    }
    
    else {
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
    
    return isDone;
  }
}
