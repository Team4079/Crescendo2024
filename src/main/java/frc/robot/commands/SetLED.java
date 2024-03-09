// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utils.GlobalsValues.LimelightValues;
import frc.robot.utils.GlobalsValues.SwerveConstants;


public class SetLED extends Command {
  /** Creates a new SetLED. */
  private final LED led;

  public SetLED(LED led) {
    this.led = led;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
        // Vision LED
    if (LimelightValues.hasTarget) {
      if (Math.abs(-LimelightValues.tx) <= SwerveConstants.limelightDeadband) {
        // Set LED to green (Based on detecting AprilTag)
        led.rainbow(SwerveConstants.greenLED[0], SwerveConstants.greenLED[1], SwerveConstants.greenLED[2]);
      } else {
        // Set LED to orange (Based on detecting AprilTag)
        led.rainbow(SwerveConstants.orangeLED[0], SwerveConstants.orangeLED[1], SwerveConstants.orangeLED[2]);
      }
    } else {
      // Remove Red LED light when in competition.
      led.rainbow(SwerveConstants.redLED[0], SwerveConstants.redLED[1], SwerveConstants.redLED[2]); // Set led to red
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
