// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SubwooferShot extends SequentialCommandGroup {
  /** Creates a new SubwooferShot. */
  private Shooter shooter;
  private Pivot pivot;
  private SwerveSubsystem swerveSubsystem;
  private Limelight limelight;

  public SubwooferShot(Shooter shooter, Pivot pivot, SwerveSubsystem swerveSubsystem, Limelight limelight) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    addRequirements(shooter, pivot, swerveSubsystem, limelight);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(new SetPivot(pivot, PivotGlobalValues.PIVOT_SUBWOOFER_ANGLE).withTimeout(0.6),
      new ShooterRampUp(shooter, limelight).withTimeout(0.6)),
        new PushRing(shooter, limelight, false).withTimeout(0.3),
        new StopShooter(shooter).withTimeout(0.1),
        new SetPivot(pivot, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(0.4));
  }
}
