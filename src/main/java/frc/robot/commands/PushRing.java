// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** The {@link PushRing} class is the command that pushes the ring into the shooter. */
public class PushRing extends Command {

  private Shooter shooter;

  /** Creates a new Shoot. */
  public PushRing(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setKrakenVelocity(ShooterGlobalValues.THROUGH_SPEED);
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
