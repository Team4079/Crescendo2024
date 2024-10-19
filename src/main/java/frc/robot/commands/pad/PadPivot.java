package frc.robot.commands.pad;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.LogitechGamingPad;

/** The {@link PadPivot} class is a command that controls the pivot using a gamepad. */
public class PadPivot extends Command {
  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  /** The gamepad used to control the pivot. */
  private final LogitechGamingPad pad;

  /**
   * Creates a new PadPivot command.
   *
   * @param pivot The Pivot subsystem used by this command.
   * @param pad The gamepad used to control the pivot.
   */
  public PadPivot(Pivot pivot, LogitechGamingPad pad) {
    this.pivot = pivot;
    this.pad = pad;
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
    // if (Math.abs(pad.getLeftTriggerValue()) > 0.01) {
    //   pivot.movePivot(-pad.getLeftTriggerValue() * 0.3);

    if (pad.getDPadDown())
    {
      pivot.movePivot(-0.1);
    }


    else if (Math.abs(pad.getRightTriggerValue()) > 0.01) {
      pivot.movePivot(pad.getRightTriggerValue() * 0.3);
    }

    else
    {
      pivot.stopMotors();
    }

    // if (pad.getDPadDown()) {
    //   new SetPivot(pivot, PivotGlobalValues.PIVOT_SOURCE).schedule();
    // }
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
