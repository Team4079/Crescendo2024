// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;

public class PassNoteGyro extends Command {
  private final SwerveSubsystem swerve;
  private final Pivot pivot;
  private final Shooter shooter;
  private final boolean isBlueSide;
  /** Creates a new PassNoteGyro. */
  public PassNoteGyro(SwerveSubsystem swerve, Pivot pivot, Shooter shooter) {
    this.swerve = swerve;
    this.pivot = pivot;
    this.shooter = shooter;
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      isBlueSide = alliance.get().equals(DriverStation.Alliance.Blue);
    } else {
      isBlueSide = true;
    }
    addRequirements(swerve, pivot, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
