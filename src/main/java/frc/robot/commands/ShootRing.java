// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.speaker.AutoAlign;
import frc.robot.commands.speaker.PivotShooterSetUp;
import frc.robot.commands.speaker.StopShooter;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootRing extends SequentialCommandGroup {
  public ShootRing(
      Shooter shooter, Pivot pivot, SwerveSubsystem swerveSubsystem, Photonvision photonvision) {
    addRequirements(shooter, pivot, swerveSubsystem, photonvision);
    addCommands(
        new ParallelCommandGroup(
            new PivotShooterSetUp(pivot, shooter, photonvision).withTimeout(0.6),
            new AutoAlign(swerveSubsystem, photonvision).withTimeout(0.6)),
        new PushRing(shooter, photonvision, false).withTimeout(0.5),
        new StopShooter(shooter).withTimeout(0.01),
        new SetPivot(pivot, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(0.4));
  }
}
