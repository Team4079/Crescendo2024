// Copyrieewght(c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.pad;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.MotorGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.LogitechGamingPad;

public class PadDrive extends Command {

  private final SwerveSubsystem swerveSubsystem;
  private final boolean isFieldOriented;
  private final LogitechGamingPad pad;

  public PadDrive(SwerveSubsystem swerveSubsystem, LogitechGamingPad pad, boolean isFieldOriented) {
    this.swerveSubsystem = swerveSubsystem;
    this.pad = pad;
    this.isFieldOriented = isFieldOriented;
    addRequirements(this.swerveSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Coordinate position = positionSet(pad);

    double rotation = -pad.getRightAnalogXAxis() * MotorGlobalValues.MAX_ANGULAR_SPEED;

    SmartDashboard.putNumber("X Jostick", position.getX());
    SmartDashboard.putNumber("Y Joystick", position.getY());

    swerveSubsystem.setDriveSpeeds(
        position.getY(), position.getX(), rotation * 0.5, isFieldOriented);
  }

  public static Coordinate positionSet(LogitechGamingPad pad) {
    double x = -pad.getLeftAnalogXAxis() * MotorGlobalValues.MAX_SPEED;
    if (Math.abs(x) < SwerveGlobalValues.xDEADZONE) {
      x = 0;
    }

    double y = -pad.getLeftAnalogYAxis() * MotorGlobalValues.MAX_SPEED;
    if (Math.abs(y) < SwerveGlobalValues.yDEADZONE) {
      y = 0;
    }

    return new Coordinate(x, y);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public record Coordinate(double x, double y) {
    public double getX() {
      return x;
    }

    public double getY() {
      return y;
    }
  }
}
