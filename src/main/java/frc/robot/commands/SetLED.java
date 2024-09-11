// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;

/**
 * The {@link SetLED} command sets the LED color based on the Limelight's vision
 * target
 */
public class SetLED extends Command {
  /** Creates a new SetLED. */
  private final LED led;

  public SetLED(LED led) {
    this.led = led;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Vision LED
    // if (ShooterGlobalValues.HAS_PIECE) {
    //   if (Math.abs(-LimelightGlobalValues.tx) <= SwerveGlobalValues.limelightDeadband) {
    //     // Set LED to green (Based on aligned with AprilTag and has piece)
    //     led.rainbow(SwerveGlobalValues.greenLED[0], SwerveGlobalValues.greenLED[1], SwerveGlobalValues.greenLED[2]);
    //   } else {
    //     // Set LED to orange (Based on not aligned with AprilTag and has piece)
    //     led.rainbow(SwerveGlobalValues.orangeLED[0], SwerveGlobalValues.orangeLED[1], SwerveGlobalValues.orangeLED[2]);
    //   }
    // } else {
      // Set LED to HighTide 4414 colors (Based on not aligned with AprilTag and doesn't have piece)
      led.setRGB(SwerveGlobalValues.HIGHTIDE_LED[0], SwerveGlobalValues.HIGHTIDE_LED[1], SwerveGlobalValues.HIGHTIDE_LED[2]);
    // }
  }

  // When it doesnt have note do hightide blue
  // When it does have a note but not aligned orange
  // When it does have a note and is aligned green

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
