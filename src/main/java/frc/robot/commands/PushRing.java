package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** The {@link PushRing} class is the command that pushes the ring into the shooter. */
public class PushRing extends Command {
  /** The Shooter subsystem used by this command. */
  private final Shooter shooter;

  /** The Limelight subsystem used by this command. */
  private final Limelight limelight;

  /** Whether the Limelight is enabled. */
  private final boolean limelightEnabled;

  /**
   * Creates a new PushRing command.
   *
   * @param shooter The Shooter subsystem used by this command.
   * @param limelight The Limelight subsystem used by this command.
   * @param limelightEnabled Whether the Limelight is enabled.
   */
  public PushRing(Shooter shooter, Limelight limelight, boolean limelightEnabled) {
    this.shooter = shooter;
    this.limelight = limelight;
    this.limelightEnabled = limelightEnabled;
    addRequirements(shooter, limelight);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    if (limelightEnabled) {
      if (limelight.getDistance() > 0.1) {
        shooter.setKrakenVelocity(ShooterGlobalValues.PUSH_RING_RPS);
      } else {
        shooter.stopAllMotors();
      }
    } else {
      shooter.setKrakenVelocity(ShooterGlobalValues.PUSH_RING_RPS);
    }
  }

  /**
   * Called once the command ends or is interrupted.
   *
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    shooter.stopKraken();
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
