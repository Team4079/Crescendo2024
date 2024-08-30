// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** The {@link LowerPivot} class is a command that lowers the pivot to its neutral position. */
public class LowerPivot extends Command {
  /** Creates a new LowerPivot. */
  private Pivot pivot;
  private double deadband;
  
  public LowerPivot(Pivot pivot) {
    pivot = this.pivot;
    deadband = 25;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!ShooterGlobalValues.HAS_PIECE) {
      if (Math.abs(pivot.getAbsoluteEncoder() - PivotGlobalValues.PIVOT_NEUTRAL_ANGLE) > deadband) {
        pivot.setMotorPosition(PivotGlobalValues.PIVOT_NEUTRAL_ANGLE, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE);
      } else {
        pivot.stopMotors();
      }
    } else {
      new PivotAlign(pivot);
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
