// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;
import frc.robot.utils.GlobalsValues.MotorGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.PID;

public class TeleOpAlign extends Command {
  /** Creates a new AutoAlign. */
  private final LogitechGamingPad pad;
  private SwerveSubsystem swerveSubsystem;
  private double x;
  private double y;
  private double rot;

  // Horizontal PID and offset
  private double horizontalError;

  // Rotation PID and offset
  private PID rotationalPID;

  public TeleOpAlign(SwerveSubsystem swerveSubsystem, LogitechGamingPad pad) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pad = pad;
    this.swerveSubsystem = swerveSubsystem;
    rotationalPID = BasePIDGlobal.rotationalPID;
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    horizontalError = -LimelightGlobalValues.tx;

    if (Math.abs(horizontalError) >= SwerveGlobalValues.limelightDeadband) {
      rot = rotationalPID.calculate(horizontalError, 0);
    }

    if (MotorGlobalValues.SLOW_MODE) {
      y = pad.getLeftAnalogXAxis() * MotorGlobalValues.MAX_SPEED * MotorGlobalValues.SLOW_SPEED;
      x = pad.getLeftAnalogYAxis() * -MotorGlobalValues.MAX_SPEED * MotorGlobalValues.SLOW_SPEED;
    } else if (MotorGlobalValues.AACORN_MODE) {
      y = pad.getLeftAnalogXAxis() * MotorGlobalValues.MAX_SPEED * MotorGlobalValues.AACORN_SPEED;
      x = pad.getLeftAnalogYAxis() * -MotorGlobalValues.MAX_SPEED * MotorGlobalValues.AACORN_SPEED;
    } else {
      y = pad.getLeftAnalogXAxis() * MotorGlobalValues.MAX_SPEED * 0.6;
      x = pad.getLeftAnalogYAxis() * -MotorGlobalValues.MAX_SPEED * 0.6;
    }

    if (Math.abs(pad.getLeftAnalogXAxis()) < SwerveGlobalValues.JOYSTICK_DEADBAND) {
      y = 0;
    }

    if (Math.abs(pad.getLeftAnalogYAxis()) < SwerveGlobalValues.JOYSTICK_DEADBAND) {
      x = 0;
    }

    swerveSubsystem.drive(x * MotorGlobalValues.SPEED_CONSTANT, y * MotorGlobalValues.SPEED_CONSTANT, rot, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}