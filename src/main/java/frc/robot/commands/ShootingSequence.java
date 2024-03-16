// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingSequence extends SequentialCommandGroup {
  /** Creates a new AutoSequence. */

  private Pivot pivot;
  private Shooter shooter;

  public ShootingSequence(Pivot pivotyboi, Shooter shootyboi) {
    this.pivot = pivotyboi;
    this.shooter = shootyboi;

    addRequirements(pivotyboi, shootyboi);

    /** Command to run shooting sequence mainly in auto */

    //Why is this not parallel command group for first two?
    addCommands(
      new ParallelCommandGroup(
        new SetPivot(pivotyboi, PivotGlobalValues.PIVOT_SUBWOOFER_ANGLE).withTimeout(1.2)
        // new ShooterRampUp(shooter, ShooterGlobalValues.SHOOTER_SPEED).withTimeout(1.2)
      ), 
      // new PushRing(shootyboi).withTimeout(0.3),
      new InstantCommand(shooter::stopAllMotors),
      new SetPivot(pivotyboi, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(1.5)
    );
  }
}
