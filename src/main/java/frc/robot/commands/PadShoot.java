// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

public class PadShoot extends Command {
  /** Creates a new PadShoot. */
  private Shooter shooter;
  private LogitechGamingPad opPad;

  public PadShoot(Shooter shooter, LogitechGamingPad opPad) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.opPad = opPad;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (opPad.getBReleased()) {
      shooter.setShooterVelocity(ShooterGlobalValues.PASSTHROUGH_RPS, ShooterGlobalValues.PASSTHROUGH_RPS);
    } else {
      shooter.stopShooter();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopKraken();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
