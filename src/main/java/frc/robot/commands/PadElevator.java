package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.ElevatorGlobalValues;

/** The {@link PadPivot} class is a command that controls the pivot using a gamepad. */
public class PadElevator extends Command {
  /** The Pivot subsystem used by this command. */
  private final Elevator elevator;

  /** The gamepad used to control the pivot. */
  private final LogitechGamingPad pad;

  /**
   * Creates a new PadPivot command.
   *
   * @param pivot The Pivot subsystem used by this command.
   * @param pad The gamepad used to control the pivot.
   */
  public PadElevator(Elevator elevator, LogitechGamingPad pad) {
    this.elevator = elevator;
    this.pad = pad;
    addRequirements(elevator);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    if (Math.abs(pad.getLeftAnalogYAxis()) > 0.15 && ElevatorGlobalValues.ELEVATOR_TEST){
      elevator.moveElevator(-pad.getLeftAnalogYAxis() * 0.75);
    }
  }

  /**
   * Returns true when the %command should end.
   *
   * @return false, as this command never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
