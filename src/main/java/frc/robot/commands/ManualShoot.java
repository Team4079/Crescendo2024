// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualShoot extends SequentialCommandGroup {
  public ManualShoot(Shooter shooter, Limelight limelight, Pivot pivot) {
    addRequirements(shooter, limelight, pivot);
    addCommands(
        new StagePassPivot(pivot).withTimeout(0.75),
        new StagePass(shooter).withTimeout(0.4414),
        new PushRing(shooter, limelight, false).withTimeout(0.5),
        new StopShooter(shooter));
  }
}
