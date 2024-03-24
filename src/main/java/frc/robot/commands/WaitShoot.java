// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WaitShoot extends SequentialCommandGroup {
  /** Creates a new WaitShoot. */
  private Shooter shooter;
  private Pivot pivot;
  private Limelight limelight;

  public WaitShoot(Shooter shooter, Pivot pivot, Limelight limelight) {
    this.shooter = shooter;
    this.pivot = pivot;
    this.limelight = limelight;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WaitCommand(2),
      new ShootingSequence(pivot, shooter, limelight)
    );
  }
}
