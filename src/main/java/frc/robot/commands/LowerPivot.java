// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.PivotConstants;
import frc.robot.utils.Constants.ShooterConstants;

public class LowerPivot extends Command {
  /** Creates a new LowerPivot. */
  private Pivot pivot;
  private double deadband;
  
  public LowerPivot(Pivot pivot) {
    pivot = this.pivot;
    deadband = 25;

    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterConstants.IS_SHOOTING = false && IntakeConstants.HAS_PEICE)
    {
      if (Math.abs(pivot.getPivotPos() - PivotConstants.PIVOT_NEUTRAL_ANGLE) > deadband) {
        pivot.setMotorPosition(PivotConstants.PIVOT_NEUTRAL_ANGLE, PivotConstants.PIVOT_NEUTRAL_ANGLE);
      }

      else {
        pivot.stopMotors();
      }
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
