// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

public class PadPivot extends Command {
  private Pivot pivot;
  private LogitechGamingPad opPad;

  /** Creates a new PadPivot. */
  public PadPivot(Pivot pivot, LogitechGamingPad opPad) {
    addRequirements(pivot);
    this.pivot = pivot;
    this.opPad = opPad;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (PivotGlobalValues.is_SOFTLIMIT) {
    //   if (pivot.getPivotLeftPos() < PivotGlobalValues.PIVOT_MIN_ANGLE) {
    //     if (opPad.getLeftAnalogYAxis() < 0) {
    //       pivot.movePivot(opPad.getLeftAnalogYAxis());
    //     } else {
    //       pivot.movePivot(0);
    //     }
    //   } else if (pivot.getPivotLeftPos() > PivotGlobalValues.PIVOT_MAX_ANGLE) {
    //     if (opPad.getLeftAnalogYAxis() > 0) {
    //       pivot.movePivot(opPad.getLeftAnalogYAxis());
    //     } else {
    //       pivot.movePivot(0);
    //     }
    //   } else {
    //     pivot.movePivot(opPad.getLeftAnalogYAxis());
    //   }
    // } 
    // else {

    if (Math.abs(opPad.getLeftAnalogYAxis()) > 0.03)
    {
      pivot.movePivot(-opPad.getLeftAnalogYAxis());

    }

    else{
      pivot.stopMotors();
    }
    // }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
