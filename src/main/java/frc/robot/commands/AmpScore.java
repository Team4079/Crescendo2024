// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** The {@link AmpScore} class is a command that sets the pivot to the amp angle and starts the shooter. */
public class AmpScore extends Command {

  private double deadband;

  private Shooter shooter;

  private Pivot pivot;

  /** Creates a new Shoot. */
  public AmpScore(Shooter shooter, Pivot pivot) {
    deadband = 5;
    this.shooter = shooter;
    this.pivot = pivot;
    addRequirements(shooter, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Sets the pivot to the amp angle
    pivot.setMotorPosition(PivotGlobalValues.PIVOT_AMP_ANGLE, PivotGlobalValues.PIVOT_AMP_ANGLE);

    // If the pivot is within the deadband, stop the pivot and start the shooter
    if (Math.abs(pivot.getPivotPos() - PivotGlobalValues.PIVOT_AMP_ANGLE) < deadband) {
      pivot.stopMotors();
      shooter.setKrakenVelocity(ShooterGlobalValues.KRAKEN_SPEED);
      shooter.setShooterVelocity(ShooterGlobalValues.SHOOTER_SPEED, -ShooterGlobalValues.SHOOTER_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(shooter.getLeftShooterVelocity() - ShooterGlobalValues.SHOOTER_SPEED) < deadband)
    {
      return true;
    }
    return false;
  }
}