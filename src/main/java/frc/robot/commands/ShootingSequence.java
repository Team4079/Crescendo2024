// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;



// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingSequence extends SequentialCommandGroup {
  /** Creates a new TeleOpShoot. */
  private SwerveSubsystem subsystem;
  private Limelight limelety;
  private LED led;
  private Pivot pivot;
  private Shooter shooter;

  public ShootingSequence(SwerveSubsystem subsystem, Limelight limelety, LED ledprobablyworking, Pivot pivotyboi, Shooter shootyboi) {
    subsystem = this.subsystem;
    limelety = this.limelety;

    /**
     * Command to run shooting sequence mainly in auto
     */
    addCommands(
        new ParallelCommandGroup(
          new AutoAlign(subsystem, limelety, led),
          new PivotAlign(pivot, limelety),
          new ShooterRampUp(shooter)
      ),
      new ShootCommands(shooter).withTimeout(1),
      new InstantCommand(shooter::stopAllMotors)
    );
  }
}
