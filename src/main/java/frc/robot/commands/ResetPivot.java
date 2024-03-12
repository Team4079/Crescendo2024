// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;;

/** The {@link ResetPivot} class is a command that resets the pivot to its neutral position. */
public class ResetPivot extends Command {

  private Pivot pivot;
  // Get distance when after we mount the limelight
  private double setPoint;
  private double deadband;
  private double timeout;

  /** Creates a new Shoot. */
  public ResetPivot(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  // // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setMotorPosition(PivotGlobalValues.PIVOT_NEUTRAL_ANGLE, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE);
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
