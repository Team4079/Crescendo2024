package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;

/** The {@link PivotAlign} class is a command that aligns the pivot to the target. */
public class PivotAlign extends Command {
  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  /** The deadband value for the pivot position. */
  private final double deadband;

  /** The timeout counter for the alignment process. */
  private double timeout;

  /**
   * Creates a new PivotAlign command.
   *
   * @param pivot The Pivot subsystem used by this command.
   */
  public PivotAlign(Pivot pivot) {
    this.pivot = pivot;
    deadband = 0.5;
    timeout = 0;
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
    // Get distance when after we mount the limelight
    double[] llValues = LimelightGlobalValues.robotPoseTargetSpace;
    double setPoint = pivot.shootPos(llValues[2]);
    pivot.setMotorPosition(pivot.shootPos(setPoint), pivot.shootPos(setPoint)); // yessica noted

    timeout = Math.abs(setPoint - pivot.getPivotPositionAvg()) < deadband ? timeout + 1 : 0;
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
   * @return true if the timeout counter has reached 10, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return timeout == 10;
  }
}
