// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PushRing;
import frc.robot.commands.SetPivot;
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
public class ShootRingAuto extends SequentialCommandGroup {
  public ShootRingAuto(
      Shooter shooter, Pivot pivot, SwerveSubsystem swerveSubsystem, Photonvision photonvision) {
    addRequirements(shooter, pivot, swerveSubsystem, photonvision);
    addCommands(
        new PivotShooterSetUp(pivot, shooter, photonvision).withTimeout(1),
        new PushRing(shooter).withTimeout(0.65),
        new StopShooter(shooter).withTimeout(0.01),
        new SetPivot(pivot, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(0.4));
  }
}
