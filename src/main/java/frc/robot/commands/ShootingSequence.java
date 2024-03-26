// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingSequence extends SequentialCommandGroup {
  /** Creates a new AutoSequence. */

  private Pivot pivot;
  private Shooter shooter;
  private Limelight limelight;
  private SwerveSubsystem swerveSubsystem;

  public ShootingSequence(Pivot pivotyboi, Shooter shootyboi, Limelight limelight, SwerveSubsystem swerveSubsystem) {
    this.pivot = pivotyboi;
    this.shooter = shootyboi;
    this.limelight = limelight;
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(pivotyboi, shootyboi, limelight, swerveSubsystem);

    /** Command to run shooting sequence mainly in auto */

    // Why is this not parallel command group for first two?

    addCommands(
        new WaitCommand(0.05),
        new PivotShooterSetUp(pivotyboi, shootyboi, limelight, swerveSubsystem).withTimeout(1.25),
        new PushRing(shootyboi, limelight).withTimeout(0.3),
        new StopShooter(shooter).withTimeout(0.1),
        new SetPivot(pivotyboi, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(0.4));
  }
}
