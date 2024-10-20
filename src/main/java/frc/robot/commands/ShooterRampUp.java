package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/** Command to ramp up the shooter to a specified velocity. */
public class ShooterRampUp extends Command {
  // Deadband value for velocity comparison.
  private final double deadband;

  // Shooter subsystem instance.
  private final Shooter shooter;

  // Desired revolutions per second for the shooter.
  private double rps;

  /**
   * Constructor for ShooterRampUp command.
   *
   * @param shooter The shooter subsystem.
   * @param photonvision The photonvision subsystem.
   */
  public ShooterRampUp(Shooter shooter) {
    addRequirements(shooter);
    deadband = 5;
    this.shooter = shooter;
  }

  /**
   * Initializes the command by calculating the desired shooter velocity and setting the shooter to
   * that velocity.
   */
  @Override
  public void initialize() {
    rps =
        ShooterGlobalValues.SHOOTER_SPEED;
    SmartDashboard.putNumber("Jessica is smart", rps);
    shooter.setShooterVelocity(-ShooterGlobalValues.SHOOTER_SPEED, -ShooterGlobalValues.SHOOTER_SPEED);
  }

  /**
   * Executes the command by checking if the shooter velocity is within the limit and updating the
   * SmartDashboard.
   */
  @Override
  public void execute() {
    shooter.setShooterVelocity(-ShooterGlobalValues.SHOOTER_SPEED, -ShooterGlobalValues.SHOOTER_SPEED);
    SmartDashboard.putBoolean(
        "Shooter Within Limit",
        Math.abs(shooter.getKrakenVelocity() - rps) < ShooterGlobalValues.RPM_THRESHOLD);
  }

  /**
   * Checks if the command is finished by comparing the current shooter velocity with the desired
   * velocity within a deadband.
   *
   * @return true if the command is finished, false otherwise.
   */
  @Override
  public boolean isFinished() {
    // return Math.abs(shooter.getLeftShooterVelocity() - ShooterGlobalValues.SHOOTER_SPEED)
    //     < deadband;
    return false;
  }
}
