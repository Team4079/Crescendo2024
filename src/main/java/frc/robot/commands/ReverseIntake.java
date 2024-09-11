package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.IntakeGlobalValues;

/** The {@link ReverseIntake} class is a command that reverses the intake and shooter mechanisms. */
public class ReverseIntake extends Command {
  /** The Intake subsystem used by this command. */
  private final Intake intake;

  /** The Shooter subsystem used by this command. */
  private final Shooter shooter;

  /**
   * Creates a new ReverseIntake command.
   *
   * @param intake The Intake subsystem used by this command.
   * @param shooter The Shooter subsystem used by this command.
   */
  public ReverseIntake(Intake intake, Shooter shooter) {
    this.intake = intake;
    this.shooter = shooter;
    addRequirements(intake, shooter);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    intake.stopKraken();
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    intake.setIntakeVelocity(-IntakeGlobalValues.REVERSE_INTAKE_SPEED);
    shooter.setKrakenVelocity(IntakeGlobalValues.REVERSE_INTAKE_SPEED);
    shooter.setShooterVelocity(
        IntakeGlobalValues.REVERSE_INTAKE_SPEED / 2, IntakeGlobalValues.REVERSE_INTAKE_SPEED / 2);
  }

  /**
   * Called once the command ends or is interrupted.
   *
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    intake.stopKraken();
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
