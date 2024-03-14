// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.LogitechGamingPad;

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
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.movePivot(opPad.getLeftAnalogYAxis());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
