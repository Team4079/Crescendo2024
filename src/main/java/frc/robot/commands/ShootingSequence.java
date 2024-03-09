// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingSequence extends SequentialCommandGroup {
  /** Creates a new TeleOpShoot. */
  private SwerveSubsystem subsystem;
  private Pivot pivot;
  private Shooter shooter;

  public ShootingSequence(SwerveSubsystem subsystem, Pivot pivotyboi, Shooter shootyboi) {
    subsystem = this.subsystem;

    /** Command to run shooting sequence mainly in auto */
    addCommands(
        new ParallelCommandGroup(
            new AutoAlign(subsystem),
            new PivotAlign(pivot),
            new ShooterRampUp(shooter)),
        new ShooterRampUp(shooter).withTimeout(1)
    // new InstantCommand(shooter::stopAllMotors)
    );
  }
}
