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

  public ShootingSequence(Pivot pivot, Shooter shooter, Limelight limelight, SwerveSubsystem swerveSubsystem) {
    this.pivot = pivot;
    this.shooter = shooter;
    this.limelight = limelight;
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(pivot, shooter, limelight, swerveSubsystem);

    /** Command to run shooting sequence mainly in auto */
    addCommands(
        new WaitCommand(0.02),
        new PivotShooterSetUpAuto(pivot, shooter, limelight, swerveSubsystem).withTimeout(1.35),
        new PushRing(shooter, limelight, false).withTimeout(0.45),
        new StopShooterAuto(shooter).withTimeout(0.02));
        // new SetPivot(pivot, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(0.3));
  }
}
