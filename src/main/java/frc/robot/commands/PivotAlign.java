// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;

/** The {@link PivotAlign} class is a command that aligns the pivot to the target. */
public class PivotAlign extends Command {

  private Pivot pivot;
  // // Get distance when after we mount the limelight
  private double[] llValues;
  private double setPoint;
  private double deadband;
  private double timeout;

  /** Creates a new Shoot. */
  public PivotAlign(Pivot pivot) {
    this.pivot = pivot;
    deadband = 0.5;
    timeout = 0;
    addRequirements(pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    llValues = LimelightGlobalValues.robotPoseTargetSpace;

    setPoint = pivot.shootPos(llValues[2]);

    pivot.setMotorPosition(pivot.shootPos(setPoint), pivot.shootPos(setPoint)); // yessica noted

    if (Math.abs(setPoint - pivot.getPivotPos()) < deadband)
    {
      timeout++;
    }
    else
    {
      timeout = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (timeout == 10)
    {
      return true;
    }
    return false;
  }
}
