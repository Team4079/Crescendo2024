// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

public class PivotPassing extends SequentialCommandGroup {
  /** Creates a new PivotPassing. */
  private Shooter shooter;
  private Pivot pivot;
  private SwerveSubsystem swerveSubsystem;
  private Limelight limelight;

  // 22 degrees left to rotate to subwoofer
  public PivotPassing(Shooter shooter, Pivot pivot, SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.pivot = pivot;
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    addRequirements(shooter, pivot, swerveSubsystem);

    addCommands(
      new ParallelCommandGroup(new SetPivot(pivot, PivotGlobalValues.PIVOT_PASS_ANGLE).withTimeout(0.6)),
      new ShooterRampUp(shooter, withTimeout(0.6)),
        new PushRing(shooter, limelight, false)
    );
  }
}
