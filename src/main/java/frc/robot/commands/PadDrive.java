package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.MotorGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.LogitechGamingPad;

/** The {@link PadDrive} command is the command that drives the robot using a gamepad. */
public class PadDrive extends Command {
  /** The SwerveSubsystem used by this command. */
  private final SwerveSubsystem swerveSubsystem;

  /** Whether the drive is field-oriented. */
  private final boolean isFieldOriented;

  /** The gamepad used to control the robot. */
  private final LogitechGamingPad pad;

  /**
   * Creates a new PadDrive command.
   *
   * @param swerveSubsystem The SwerveSubsystem used by this command.
   * @param pad The gamepad used to control the robot.
   * @param isFieldOriented Whether the drive is field-oriented.
   */
  public PadDrive(SwerveSubsystem swerveSubsystem, LogitechGamingPad pad, boolean isFieldOriented) {
    this.swerveSubsystem = swerveSubsystem;
    this.pad = pad;
    this.isFieldOriented = isFieldOriented;
    addRequirements(this.swerveSubsystem);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    Coordinate position = positionSet(pad);
    double rotation = pad.getRightAnalogXAxis();

    double turn = 0;

    if (Math.abs(rotation) < 0.05) {
      rotation = 0;
    }

    SmartDashboard.putNumber("X axis joystick", position.getX());
    SmartDashboard.putNumber("y axis joystick", position.getY());

    turn = rotation * MotorGlobalValues.MAX_ANGULAR_SPEED * 2 * 1.5; // magic numbers

    if (MotorGlobalValues.AACORN_MODE) {
      swerveSubsystem.getDriveSpeeds(
          -position.getX() * MotorGlobalValues.AACORN_SPEED,
          position.getY() * MotorGlobalValues.AACORN_SPEED,
          turn,
          isFieldOriented);
    } else {
      swerveSubsystem.getDriveSpeeds(
          -position.getX() * MotorGlobalValues.SPEED_CONSTANT,
          position.getY() * MotorGlobalValues.SPEED_CONSTANT,
          turn,
          isFieldOriented);
    }
  }

  /**
   * Called once the command ends or is interrupted.
   *
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    // No specific action needed when the command ends.
  }

  /**
   * Returns true when the command should end.
   *
   * @return false, as this command never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * Sets the position based on the gamepad input.
   *
   * @param pad The gamepad used to control the robot.
   * @return The coordinate representing the position.
   */
  public static Coordinate positionSet(LogitechGamingPad pad) {
    double x;
    double y;
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
    return new Coordinate(x, y);
  }

  /** A record representing a coordinate with x and y values. */
  public record Coordinate(double x, double y) {
    /**
     * Gets the x value of the coordinate.
     *
     * @return The x value.
     */
    public double getX() {
      return x;
    }

    /**
     * Gets the y value of the coordinate.
     *
     * @return The y value.
     */
    public double getY() {
      return y;
    }
  }
}
