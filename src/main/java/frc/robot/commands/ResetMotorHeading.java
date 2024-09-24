package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;
import frc.robot.utils.PID;

/** The {@link ResetMotorHeading} command resets the heading of the swerve modules to 0. */
public class ResetMotorHeading extends Command {
  /** The SwerveSubsystem used by this command. */
  private final SwerveSubsystem swerveSubsystem;

  /** The deadband value for the heading error. */
  private final double deadband;

  /** The PID controller for heading adjustment. */
  private final PID pid;

  /**
   * Creates a new ResetMotorHeading command.
   *
   * @param swerveSubsystem The SwerveSubsystem used by this command.
   */
  public ResetMotorHeading(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    pid = BasePIDGlobal.ROTATIONAL_PID;
    deadband = 5;
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
    double error = swerveSubsystem.getHeading();
    if (Math.abs(error) > deadband) {
      swerveSubsystem.getDriveSpeeds(0, 0, pid.calculate(error, 0), false);
    } else {
      swerveSubsystem.stop();
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
}
