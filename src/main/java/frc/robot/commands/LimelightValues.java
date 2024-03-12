// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;

/** The {@link LimelightValues} class is a command that gets the values from the limelight. */
public class LimelightValues extends Command {
  /** Creates a new LimelightGlobalValues. */
  private Limelight limelety;

  public LimelightValues(Limelight limelight) {
    this.limelety = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.limelety);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightGlobalValues.tx = limelety.getTx();
    LimelightGlobalValues.ty = limelety.getTy();
    LimelightGlobalValues.ta = limelety.getTa();
    LimelightGlobalValues.tv = limelety.getTv();

    LimelightGlobalValues.robotPoseTargetSpace = limelety.getRobotPose_TargetSpace2D();
    LimelightGlobalValues.tagIDAvailable = limelety.getTag();
    LimelightGlobalValues.hasTarget = limelety.isTarget();
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
