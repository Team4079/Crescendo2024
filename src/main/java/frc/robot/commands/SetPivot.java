// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;;

/** The {@link ResetPivot} class is a command that resets the pivot to its neutral position. */
public class SetPivot extends Command {

  private Pivot pivot;
  private double pos;
  private PIDController pidController;
  private double velocity;
  // Get distance when after we mount the limelight

  /** Creates a new Shoot. */
  public SetPivot(Pivot pivot, double pos) {
    this.pivot = pivot;
    this.pos = pos;
    pidController = new PIDController(0.0017, 0, 0.000005);
    addRequirements(pivot);
  }

  // // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
    // pidController.setTolerance(50);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = pidController.calculate(pivot.getAbsoluteEncoder(), pos);
    SmartDashboard.putNumber("Error Pivot", -pivot.getAbsoluteEncoder() + pos);
    SmartDashboard.putNumber("Setpoint", pos);

    if (Math.abs(-pivot.getAbsoluteEncoder() + pos) < 10)
    {
       pivot.stopMotors();
    }

    else{
      pivot.movePivot(-velocity);
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
