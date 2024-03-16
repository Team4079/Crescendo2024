// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.IntakeGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/**
 * The {@link ResetPivot} class is a command that resets the pivot to its
 * neutral position.
 */
public class StartIntake extends Command {

  private Intake intake;
  private Shooter shooter;
  private Timer timer;
  private boolean isDone;

  // Get distance when after we mount the limelight

  /** Creates a new Shoot. */
  public StartIntake(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
    timer = new Timer();
    isDone = false;

    addRequirements(intake, shooter);
  }

  // // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeVelocity(IntakeGlobalValues.INTAKE_SPEED);
    shooter.setKrakenVelocity(ShooterGlobalValues.PASSTHROUGH_RPS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ShooterGlobalValues.HAS_PIECE) {
      timer.start();

      while (timer.get() < 0.3) {
        // shooter.setShooterVelocity(6, 6);
        shooter.setKrakenVelocity(ShooterGlobalValues.PASSTHROUGH_RPS);
      }
      shooter.stopKraken();
      while (timer.get() < 0.5) {
        shooter.setKrakenVelocity(20);
      }

      shooter.stopKraken();
      isDone = true;
      intake.stopKraken();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
