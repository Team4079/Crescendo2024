// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

public class PadPivot extends Command {
  private Pivot pivot;
  private LogitechGamingPad pad;

  /** Creates a new PadPivot. */
  public PadPivot(Pivot pivot, LogitechGamingPad pad) {
    addRequirements(pivot);
    this.pivot = pivot;
    this.pad = pad;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(pad.getLeftTriggerValue()) > 0.01)
    {
      pivot.movePivot(-pad.getLeftTriggerValue() * 0.3);
    }
    else if (Math.abs(pad.getRightTriggerValue()) > 0.01) {
      pivot.movePivot(pad.getRightTriggerValue() * 0.3);
    }
    else{
      pivot.stopMotors();
    }

    // if (pad.getDPadDown())
    // {
    //   new SetPivot(pivot, PivotGlobalValues.PIVOT_SOURCE).schedule();
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
