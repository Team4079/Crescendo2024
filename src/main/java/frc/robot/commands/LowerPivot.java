package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** The {@link LowerPivot} class is a command that lowers the pivot to its neutral position. */
public class LowerPivot extends Command {
  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  /** The deadband value for the pivot position. */
  private final double deadband;

  /**
   * Creates a new LowerPivot command.
   *
   * @param pivot The Pivot subsystem used by this command.
   */
  public LowerPivot(Pivot pivot) {
    this.pivot = pivot;
    deadband = 25;
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
    if (!ShooterGlobalValues.HAS_PIECE) {
      if (Math.abs(pivot.getPivotPos() - PivotGlobalValues.PIVOT_NEUTRAL_ANGLE) > deadband) {
        pivot.setMotorPosition(
            PivotGlobalValues.PIVOT_NEUTRAL_ANGLE, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE);
      } else {
        pivot.stopMotors();
      }
    } else {
      new PivotAlign(pivot);
    }
  }

  /**
   * Called once the command ends or is interrupted.
   *
   * @param interrupted Whether the command was interrupted/canceled.
   */
  @Override
  public void end(boolean interrupted) {
    pivot.stopMotors();
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
