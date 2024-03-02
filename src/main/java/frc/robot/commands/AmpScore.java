// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Constants.PivotConstants;
import frc.robot.utils.Constants.ShooterConstants;

public class AmpScore extends Command {

  // private double deadband;

  // private Shooter shootyboi;

  // private Pivot pivot;

  /** Creates a new Shoot. */
  public AmpScore(Shooter shootyboi, Pivot pivot) {
    // deadband = 5;
    // this.shootyboi = shootyboi;
    // this.pivot = pivot;
    // addRequirements(shootyboi, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // pivot.setPosition(PivotConstants.PIVOT_AMP_ANGLE);

    // if (Math.abs(pivot.getPosition() - PivotConstants.PIVOT_AMP_ANGLE < deadband)) {
    //   pivot.stopMotors();
    //   shootyboi.setKrakenVelocity(ShooterConstants.KRAKEN_SPEED);
    //   shootyboi.setShooterVelocity(ShooterConstants.SHOOTER_SPEED, -ShooterConstants.SHOOTER_SPEED);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (Math.abs(shootyboi.getLeftShooterVelocity() - ShooterConstants.SHOOTER_SPEED) < deadband)
    // {
    //   return true;
    // }
    return false;
  }
}
