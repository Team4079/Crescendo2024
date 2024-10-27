// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class ManualShoot extends SequentialCommandGroup {
  public ManualShoot(Shooter shooter, Pivot pivot) {
    addRequirements(shooter, pivot);
    addCommands(
      new ShooterRampUp(shooter).withTimeout(0.5),
      new PushRing(shooter).withTimeout(0.5),
      new StopShooter(shooter)
    );
  }
}