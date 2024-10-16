// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.speaker;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PushRing;
import frc.robot.commands.stage.StagePass;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ManualShoot extends SequentialCommandGroup {
  public ManualShoot(
      SwerveSubsystem swerve, Shooter shooter, Photonvision photonvision, Pivot pivot) {
    addRequirements(swerve, shooter, pivot);
    addCommands(
        new StagePass(shooter).withTimeout(0.4414),
        new PushRing(shooter, photonvision, false).withTimeout(0.5),
        new StopShooter(shooter));
  }
}
