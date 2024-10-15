package frc.robot.commands.pad;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootRing;
import frc.robot.commands.speaker.ManualShoot;
import frc.robot.commands.stage.PassNoteGyro;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;

/** The {@link PadShoot} class is a command that controls the shooter using a gamepad. */
public class PadShoot extends Command {
  /** The Shooter subsystem used by this command. */
  private final Shooter shooter;

  /** The gamepad used to control the shooter. */
  private final LogitechGamingPad pad;

  /** The Limelight subsystem used by this command. */
  private final Photonvision photonvision;

  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  private final SwerveSubsystem swerve;

  /**
   * Creates a new PadShoot command.
   *
   * @param shooter The Shooter subsystem used by this command.
   * @param pad The gamepad used to control the shooter.
   * @param photonvision The photonvision subsystem used by this command.
   * @param pivot The Pivot subsystem used by this command.
   */
  public PadShoot(
      Shooter shooter,
      SwerveSubsystem swerve,
      LogitechGamingPad pad,
      Photonvision photonvision,
      Pivot pivot) {
    this.shooter = shooter;
    this.pad = pad;
    this.photonvision = photonvision;
    this.pivot = pivot;
    this.swerve = swerve;
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
      new ManualShoot(swerve, shooter, photonvision, pivot).schedule();
    }
    //    } else {
    //      shooter.stopShooter();
    //    }

    if (pad.getDPadRight()) {
      new ShootRing(shooter, pivot, swerve, photonvision).schedule(); // Uses Alt
      //    } else {
      //      shooter.stopShooter();
    }

    if (pad.getLeftTriggerValue() > 0.5) {
      new PassNoteGyro(swerve, pivot, shooter).schedule();
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
