// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** The {@link PushRingAmp} class is the command that pushes the ring into the shooter. */
public class PushRingAmp extends Command {

  private Shooter shooter;
  private boolean isDone;
  private Limelight limelight;

  /** Creates a new Shoot. */
  public PushRingAmp(Shooter shooter, Limelight limelight) {
    this.shooter = shooter;
    isDone = false;
    this.limelight = limelight;
    addRequirements(shooter, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shooter.setKrakenVelocity(-30);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setKrakenVelocity(ShooterGlobalValues.PUSH_RING_AMP_RPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooter.stopKraken();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
