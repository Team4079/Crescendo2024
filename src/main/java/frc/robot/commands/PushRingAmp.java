package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** The {@link PushRingAmp} class is the command that pushes the ring into the shooter. */
public class PushRingAmp extends Command {
  /** The Shooter subsystem used by this command. */
  private final Shooter shooter;

  /**
   * Creates a new PushRingAmp command.
   *
   * @param shooter The Shooter subsystem used by this command.
   * @param limelight The photonvision subsystem used by this command.
   */
  public PushRingAmp(Shooter shooter, Photonvision photonvision) {
    this.shooter = shooter;
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
    shooter.setKrakenVelocity(ShooterGlobalValues.PUSH_RING_AMP_RPS);
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
