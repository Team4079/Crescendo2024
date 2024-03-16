// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.IntakeGlobalValues;

public class ReverseIntake extends Command {
  /** Creates a new ReverseIntake. */
  private Intake intake;
  private Shooter shooter;

  public ReverseIntake(Intake intake, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopKraken();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeVelocity(-IntakeGlobalValues.REVERSE_INTAKE_SPEED);
    shooter.setKrakenVelocity(IntakeGlobalValues.REVERSE_INTAKE_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopKraken();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
