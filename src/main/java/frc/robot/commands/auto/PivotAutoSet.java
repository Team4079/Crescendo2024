package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SetPivot;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Pivot;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

/** The {@link PivotAutoSet} class is a command that resets the pivot to its neutral position. */
public class PivotAutoSet extends Command {
  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  /** The target position for the pivot. */
  private double pos;

  /** The PID controller for the pivot. */
  private final PIDController pidController;

  /** The timer used to determine when the command is done. */
  private final Timer timer;

  /** The deadband value for the pivot position. */
  private double deadband;

  /** Whether the command is done. */
  private boolean isDone;

  /** The Limelight subsystem used by this command. */
  private final Photonvision photonvision;

  /**
   * Creates a new PivotAutoSet command.
   *
   * @param pivot The Pivot subsystem used by this command.
   * @param limelight The Limelight subsystem used by this command.
   */
  public PivotAutoSet(Pivot pivot, Photonvision photonvision) {
    this.pivot = pivot;
    this.photonvision = photonvision;
    timer = new Timer();
    pidController = new PIDController(0.00825, 0.000000, 0.00035);
    addRequirements(pivot);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    deadband = 0.1;
    isDone = false;
    pos = PivotGlobalValues.PIVOT_SUBWOOFER_ANGLE;
    pos = photonvision.getPivotPosition();
    // pidController.setTolerance(50);
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    double velocity = pidController.calculate(pivot.getPivotPositionAvg(), pos);
    SetPivot.motorPivot(velocity, pivot, pos, deadband);

    if (Math.abs(pivot.getPivotPositionAvg() - pos) <= deadband) {
      timer.start();
      if (timer.get() >= 0.1) {
        isDone = true;
      }
    } else {
      timer.reset();
      isDone = false;
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
   * @return true if the command is done, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
