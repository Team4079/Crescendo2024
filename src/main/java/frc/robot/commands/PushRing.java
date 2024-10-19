package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** The {@link PushRing} class is the command that pushes the ring into the shooter. */
public class PushRing extends Command {
  /** The Shooter subsystem used by this command. */
  private final Shooter shooter;

  /** The Limelight subsystem used by this command. */
  private final Photonvision photonvision;

  /** Whether the Limelight is enabled. */
  private final boolean photonEnabled;

  /**
   * Creates a new PushRing command.
   *
   * @param shooter The Shooter subsystem used by this command.
   * @param limelight The Limelight subsystem used by this command.
   * @param photonEnabled Whether the Limelight is enabled.
   */
  public PushRing(Shooter shooter, Photonvision photonvision, boolean photonEnabled) {
    this.shooter = shooter;
    this.photonvision = photonvision;
    this.photonEnabled = photonEnabled;
    addRequirements(shooter);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    // if (photonEnabled) {
    //   if (photonvision.getRange(photonvision.getBestCamera()) > 0.1) {
    //     shooter.setKrakenVelocity(ShooterGlobalValues.PUSH_RING_RPS);
    //   } else {
    //     shooter.stopAllMotors();
    //   }
    // } else {
    //   shooter.setKrakenVelocity(ShooterGlobalValues.PUSH_RING_RPS);
    // }
    shooter.setKrakenVelocity(ShooterGlobalValues.PUSH_RING_RPS);
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
