package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;
import frc.robot.utils.GlobalsValues.MotorGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.PID;

/**
 * The {@link TeleOpAlign} class is a command that aligns the robot during teleoperation using the
 * Limelight and a gamepad.
 */
public class TeleOpAlign extends Command {
  /** The gamepad used to control the robot. */
  private final LogitechGamingPad pad;

  /** The SwerveSubsystem used by this command. */
  private final SwerveSubsystem swerveSubsystem;

  /** The rotation value calculated by the PID controller. */
  private double rot;

  /** The PID controller for rotational alignment. */
  private final PID rotationalPID;

  /**
   * Creates a new TeleOpAlign command.
   *
   * @param swerveSubsystem The SwerveSubsystem used by this command.
   * @param pad The gamepad used to control the robot.
   */
  public TeleOpAlign(SwerveSubsystem swerveSubsystem, LogitechGamingPad pad) {
    this.pad = pad;
    this.swerveSubsystem = swerveSubsystem;
    rotationalPID = BasePIDGlobal.ROTATIONAL_PID;
    addRequirements(swerveSubsystem);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    // Horizontal PID and offset
    double horizontalError = -LimelightGlobalValues.tx;

    if (Math.abs(horizontalError) >= SwerveGlobalValues.LIMELIGHT_DEADBAND) {
      rot = rotationalPID.calculate(horizontalError, 0);
    }

    PadDrive.Coordinate position = PadDrive.positionSet(pad);

    swerveSubsystem.getDriveSpeeds(
        position.getX() * MotorGlobalValues.SPEED_CONSTANT,
        position.getY() * MotorGlobalValues.SPEED_CONSTANT,
        rot,
        true);
  }

  /**wq
   * Called once the command ends or is interrupted.
   *
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
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
}
