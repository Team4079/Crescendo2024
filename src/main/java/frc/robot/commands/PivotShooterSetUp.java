package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

/**
 * The {@link PivotShooterSetUp} class is a command that sets up the pivot and shooter subsystems
 * for shooting.
 */
public class PivotShooterSetUp extends Command {
  /** The Pivot subsystem used by this command. */
  private final Pivot pivot;

  /** The Shooter subsystem used by this command. */
  private final Shooter shooter;

  /** The Limelight subsystem used by this command. */
  private final Photonvision photonvision;

  /** The Timer used to manage timing within the command. */
  private final Timer timer;

  /** The deadband value for the pivot position. */
  private double deadband;

  /** The target position for the pivot. */
  private double pos;

  /** The SwerveSubsystem used by this command. */
  private final SwerveSubsystem swerveSubsystem;

  /** The PID controller for rotational alignment. */
  private final PIDController rotationalController;

  /** The PID controller for velocity control. */
  private final PIDController velocityPIDController;

  /**
   * Creates a new PivotShooterSetUp command.
   *
   * @param pivot The Pivot subsystem used by this command.
   * @param shooter The Shooter subsystem used by this command.
   * @param limelight The Limelight subsystem used by this command.
   * @param swerveSubsystem The SwerveSubsystem used by this command.
   */
  public PivotShooterSetUp(
      Pivot pivot, Shooter shooter, Photonvision photonvision, SwerveSubsystem swerveSubsystem) {
    addRequirements(pivot, shooter, photonvision, swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    timer = new Timer();
    rotationalController =
        new PIDController(
            BasePIDGlobal.ROTATIONAL_PID.p,
            BasePIDGlobal.ROTATIONAL_PID.i,
            BasePIDGlobal.ROTATIONAL_PID.d);
    velocityPIDController = new PIDController(0.00825, 0.000000, 0.00035);
    rotationalController.setTolerance(3);
    this.pivot = pivot;
    this.photonvision = photonvision;
    this.shooter = shooter;
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    deadband = 0.1;
    pos =
        photonvision.getDistanceSubwoofer() < 1.5
            ? PivotGlobalValues.PIVOT_SUBWOOFER_ANGLE
            : photonvision.getPivotPosition();
    double rps =
        ShooterGlobalValues.SHOOTER_SPEED + (photonvision.getDistanceSubwoofer() - 1.5) * 5;
    shooter.setShooterVelocity(-rps, -rps);
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    double velocity = velocityPIDController.calculate(pivot.getPivotPos(), pos);
    SmartDashboard.putNumber("Error Pivot Right", -pivot.getPivotRightPos() + pos);
    SmartDashboard.putNumber("Error Pivot Left", -pivot.getPivotLeftPos() + pos);

    // Horizontal PID and offset
    double horizontalError = -photonvision.getYaw() + photonvision.getOffset();
    System.out.println(horizontalError);
    if (Math.abs(horizontalError) >= SwerveGlobalValues.LIMELIGHT_DEADBAND) {
      swerveSubsystem.setDriveSpeeds(
          0, 0, rotationalController.calculate(horizontalError, 0), false);
    } else {
      swerveSubsystem.stop();
    }

    if (Math.abs(pivot.getPivotPos() - pos) < deadband) {
      pivot.stopMotors();
    } else {
      pivot.movePivot(velocity);
    }

    if (Math.abs(pivot.getPivotPositionAvg() - pos) <= deadband) {
      timer.start();
    } else {
      timer.reset();
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
    swerveSubsystem.stop();
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
