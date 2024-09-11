package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;

/** The {@link LimelightValues} class is a command that gets the values from the limelight. */
public class LimelightValues extends Command {
  /** The Limelight subsystem used by this command. */
  private final Limelight limelety;

  /**
   * Creates a new LimelightValues command.
   *
   * @param limelight The Limelight subsystem used by this command.
   */
  public LimelightValues(Limelight limelight) {
    this.limelety = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.limelety);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    // No specific action needed when the command is initialized.
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    updateLimelightValues(this.limelety);
  }

  /**
   * Static method to update LimelightGlobalValues.
   *
   * @param limelight The Limelight subsystem used to get the values.
   */
  public static void updateLimelightValues(Limelight limelight) {
    LimelightGlobalValues.tx = limelight.getTx();
    LimelightGlobalValues.ty = limelight.getTy();
    LimelightGlobalValues.ta = limelight.getTa();
    LimelightGlobalValues.tv = limelight.getTv();

    LimelightGlobalValues.robotPoseTargetSpace = limelight.getRobotPose_TargetSpace2D();
    LimelightGlobalValues.tagIDAvailable = limelight.getTag();
    LimelightGlobalValues.hasTarget = limelight.isTarget();
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
