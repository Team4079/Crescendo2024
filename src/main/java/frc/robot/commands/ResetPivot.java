package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

/** The {@link ResetPivot} class is a command that resets the pivot to its neutral position. */
public class ResetPivot extends Command {
  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  /**
   * Creates a new ResetPivot command.
   *
   * @param pivot The Pivot subsystem used by this command.
   */
  public ResetPivot(Pivot pivot) {
    this.pivot = pivot;
    addRequirements(pivot);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    pivot.setMotorPosition(
        PivotGlobalValues.PIVOT_NEUTRAL_ANGLE, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE);
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
