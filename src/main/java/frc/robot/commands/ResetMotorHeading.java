// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.PID;
import frc.robot.utils.Constants.SwerveConstants.BasePIDConstants;

public class ResetMotorHeading extends Command {
  /** Creates a new ResetMotorHeading. */
  private SwerveSubsystem swerveSubsystem;
  private double deadband;
  private double error;
  private PID pid;
  
  public ResetMotorHeading(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    pid = BasePIDConstants.rotationalPID;
    deadband = 5;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = swerveSubsystem.pgetHeading();

    if (Math.abs(error) > deadband) {
      swerveSubsystem.drive(0, 0, pid.calculate(error, 0), false);
    } else {
      swerveSubsystem.stopModules();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
