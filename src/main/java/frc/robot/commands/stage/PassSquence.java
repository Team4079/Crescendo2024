// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.stage;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.stage.PassNoteGyro;
import frc.robot.commands.PushRing;
import frc.robot.commands.SetPivot;
import frc.robot.commands.speaker.StopShooter;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PassSquence extends SequentialCommandGroup {
  public PassSquence(
      SwerveSubsystem swerve, Shooter shooter, Photonvision photonvision, Pivot pivot, LogitechGamingPad pad) {
    addRequirements(swerve, shooter, pivot);
    addCommands(
      new ParallelCommandGroup(

        new ParallelCommandGroup(

          new StagePassPivot(pivot).withTimeout(0.75),
          new StagePass(shooter).withTimeout(0.4414)),

        new PassNoteGyro(swerve, pad).withTimeout(0.75)),   

      new PushRing(shooter).withTimeout(0.5),
      new SetPivot(pivot, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(0.4),
      new StopShooter(shooter)
    );
  }
}