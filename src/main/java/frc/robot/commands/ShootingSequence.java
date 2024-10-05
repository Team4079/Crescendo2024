// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Photonvision;
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
  private Photonvision photonvision;
  private SwerveSubsystem swerveSubsystem;

  public ShootingSequence(Pivot pivot, Shooter shooter, Photonvision photonvision, SwerveSubsystem swerveSubsystem) {
    this.pivot = pivot;
    this.shooter = shooter;
    this.photonvision = photonvision;
    this.swerveSubsystem = swerveSubsystem;

    addRequirements(pivot, shooter, swerveSubsystem);

    /** Command to run shooting sequence mainly in auto */
    addCommands(
        new WaitCommand(0.02),
        new PivotShooterSetUpAuto(pivot, shooter, photonvision, swerveSubsystem).withTimeout(1.35),
        new PushRing(shooter, photonvision, false).withTimeout(0.45),
        new StopShooterAuto(shooter).withTimeout(0.02));
        // new SetPivot(pivot, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(0.3));
  }
}
