package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.LogitechGamingPad;

/** The {@link PadShoot} class is a command that controls the shooter using a gamepad. */
public class PadShoot extends Command {
  /** The Shooter subsystem used by this command. */
  private final Shooter shooter;

  /** The gamepad used to control the shooter. */
  private final LogitechGamingPad pad;

  /** The Limelight subsystem used by this command. */
  private final Limelight limelight;

  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  /**
   * Creates a new PadShoot command.
   *
   * @param shooter The Shooter subsystem used by this command.
   * @param pad The gamepad used to control the shooter.
   * @param limelight The Limelight subsystem used by this command.
   * @param pivot The Pivot subsystem used by this command.
   */
  public PadShoot(Shooter shooter, LogitechGamingPad pad, Limelight limelight, Pivot pivot) {
    this.shooter = shooter;
    this.pad = pad;
    this.limelight = limelight;
    this.pivot = pivot;
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
    // if (opPad.getRightBumperReleased()) {
    //   shooter.setShooterVelocity(ShooterGlobalValues.PASSTHROUGH_RPS,
    // ShooterGlobalValues.PASSTHROUGH_RPS);
    // } else {
    //   shooter.stopShooter();
    // }

    if (pad.getDPadUp()) {
      new ManualShoot(shooter, limelight, pivot).schedule();
    } else {
      shooter.stopShooter();
    }

    if (pad.getDPadRight()) {
      new AmpScoreAlt(shooter, pivot, limelight).schedule(); // Uses Alt
    } else {
      shooter.stopShooter();
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
