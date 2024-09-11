package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utils.GlobalsValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;

/** The {@link SetLED} command sets the LED color based on the Limelight's vision target. */
public class SetLED extends Command {
  /** The LED subsystem used by this command. */
  private final LED led;

  /**
   * Creates a new SetLED command.
   *
   * @param led The LED subsystem used by this command.
   */
  public SetLED(LED led) {
    this.led = led;
    addRequirements(led);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    if (GlobalsValues.ShooterGlobalValues.HAS_PIECE) {
      if (Math.abs(-GlobalsValues.LimelightGlobalValues.tx)
          <= SwerveGlobalValues.LIMELIGHT_DEADBAND) {
        led.setGreen();
      } else {
        led.setOrange();
      }
    } else {
      led.setHighTide();
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
   * @return false, as this command never finishes on its own.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
