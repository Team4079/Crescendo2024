// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Constants.ShooterConstants;

public class ShooterRampUp extends Command {

  private double deadband;

  private Shooter shooter;

  /** Creates a new Shoot. */
  public ShooterRampUp(Shooter shooter) {
    deadband = 5;
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shooter.setShooterVelocity(ShooterConstants.SHOOTER_SPEED, -ShooterConstants.SHOOTER_SPEED);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (Math.abs(shooter.getLeftShooterVelocity() - ShooterConstants.SHOOTER_SPEED) < deadband)
    // {
    //   return true;
    // }
    return false;
  }
}
