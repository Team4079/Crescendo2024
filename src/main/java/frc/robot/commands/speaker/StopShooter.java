package frc.robot.commands.speaker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * Command to stop the shooter subsystem.
 */
public class StopShooter extends Command {
  /** Shooter subsystem instance. */
  private final Shooter shooter;

  /** Flag to indicate if the command is finished. */
  private boolean isFinished = false;

  /**
   * Constructor for StopShooter command.
   *
   * @param shooter The shooter subsystem.
   */
  public StopShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  /**
   * Initializes the command by stopping all motors in the shooter subsystem
   * and setting the isFinished flag to true.
   */
  @Override
  public void initialize() {
    shooter.stopAllMotors();
    isFinished = true;
  }

  /**
   * Checks if the command is finished.
   *
   * @return true if the command is finished, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
