package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The {@link PulseDown} class is a command that controls the shooter and intake subsystems to
 * perform a pulsing action.
 */
public class PulseDown extends Command {
  /** The Intake subsystem used by this command. */
  private final Intake intake;

  /** The Shooter subsystem used by this command. */
  private final Shooter shooter;

  /** The Timer used to manage timing within the command. */
  private final Timer timer;

  /** Whether the command is done. */
  private boolean isDone;

  /**
   * Creates a new PulseDown command.
   *
   * @param intake The Intake subsystem used by this command.
   * @param shooter The Shooter subsystem used by this command.
   */
  public PulseDown(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
    timer = new Timer();
    isDone = false;
    addRequirements(intake, shooter);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    if (shooter.getRingSensor()) {
      timer.start();
      while (timer.get() < 0.2) {
        shooter.setKrakenVelocity(-35);
      }
      while (timer.get() < 0.4) {
        shooter.setKrakenVelocity(30);
      }
      shooter.stopKraken();
      shooter.stopShooter();
      intake.stopKraken();
      timer.stop();
      timer.reset();
      isDone = true;
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
   * @return true if the command is done, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
