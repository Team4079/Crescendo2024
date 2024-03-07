// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants.MotorConstants;
import frc.robot.utils.Constants.SwerveConstants;
import frc.robot.utils.Constants.SwerveConstants.BasePIDConstants;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.PID;

public class TeleOpAlign extends Command {
  /** Creates a new AutoAlign. */
  private final LogitechGamingPad pad;
  private Limelight limelight;
  private SwerveSubsystem swerveSubsystem;
  private LED led;
  private double x;
  private double y;
  private double rot;

  // Horizontal PID and offset
  private double horizontalError;

  // Rotation PID and offset
  private PID rotationalPID;

  private double timeout = 0;

  public TeleOpAlign(SwerveSubsystem swerveSubsystem, Limelight limelight, LED led, LogitechGamingPad pad) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pad = pad;
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    this.led = led;
    rotationalPID = BasePIDConstants.rotationalPID;
    addRequirements(swerveSubsystem, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    horizontalError = -limelight.getTx();


    if (Math.abs(horizontalError) >= SwerveConstants.limelightDeadband) {
      rot = rotationalPID.calculate(horizontalError, 0);
    }

    if (MotorConstants.SLOW_MODE) {
      y = pad.getLeftAnalogXAxis() * MotorConstants.MAX_SPEED * MotorConstants.SLOW_SPEED;
      x = pad.getLeftAnalogYAxis() * -MotorConstants.MAX_SPEED * MotorConstants.SLOW_SPEED;
    } else if (MotorConstants.AACORN_MODE) {
      y = pad.getLeftAnalogXAxis() * MotorConstants.MAX_SPEED * MotorConstants.AACORN_SPEED;
      x = pad.getLeftAnalogYAxis() * -MotorConstants.MAX_SPEED * MotorConstants.AACORN_SPEED;
    } else {
      y = pad.getLeftAnalogXAxis() * MotorConstants.MAX_SPEED * 0.6;
      x = pad.getLeftAnalogYAxis() * -MotorConstants.MAX_SPEED * 0.6;
    }

    if (Math.abs(pad.getLeftAnalogXAxis()) < SwerveConstants.JOYSTICK_DEADBAND) {
      y = 0;
    }

    if (Math.abs(pad.getLeftAnalogYAxis()) < SwerveConstants.JOYSTICK_DEADBAND) {
      x = 0;
    }

    // Vision LED
    if (limelight.isTarget()) {
      if (Math.abs(horizontalError) <= SwerveConstants.limelightDeadband) {
        // Set LED to green (Based on detecting AprilTag)
        led.rainbow(SwerveConstants.greenLED[0], SwerveConstants.greenLED[1], SwerveConstants.greenLED[2]);
        timeout++;
      } else {
        // Set LED to orange (Based on detecting AprilTag)
        led.rainbow(SwerveConstants.orangeLED[0], SwerveConstants.orangeLED[1], SwerveConstants.orangeLED[2]);
      }
    } else {
      // Remove Red LED light when in competition.
      led.rainbow(SwerveConstants.redLED[0], SwerveConstants.redLED[1], SwerveConstants.redLED[2]); // Set led to red
      timeout = 0;
    }

    swerveSubsystem.drive(x * MotorConstants.SPEED_CONSTANT, y * MotorConstants.SPEED_CONSTANT, rot, false);
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