// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;


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
    if (LimelightGlobalValues.hasTarget) {
      if (Math.abs(-LimelightGlobalValues.tx) <= SwerveGlobalValues.limelightDeadband) {
        // Set LED to green (Based on detecting AprilTag)
        led.rainbow(SwerveGlobalValues.greenLED[0], SwerveGlobalValues.greenLED[1], SwerveGlobalValues.greenLED[2]);
      } else {
        // Set LED to orange (Based on detecting AprilTag)
        led.rainbow(SwerveGlobalValues.orangeLED[0], SwerveGlobalValues.orangeLED[1], SwerveGlobalValues.orangeLED[2]);
      }
    } else {
      // Remove Red LED light when in competition.
      led.rainbow(SwerveGlobalValues.redLED[0], SwerveGlobalValues.redLED[1], SwerveGlobalValues.redLED[2]); // Set led to red
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
