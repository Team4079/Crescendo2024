// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
// import frc.robot.utils.GlobalsValues.ShooterConstants;
// import frc.robot.utils.GlobalsValues;
// import frc.robot.utils.GlobalsValues.ShooterGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

public class AmpRampUp extends Command {

  private double deadband;

  private Shooter shooter;

  private double rps;
  private Limelight limelight;

  /** Creates a new Shoot. */
  public AmpRampUp(Shooter shooter, Limelight limelight) {
    deadband = 5;
    this.shooter = shooter;
    // this.rps = rps;
    this.limelight = limelight;
    addRequirements(shooter, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rps = ShooterGlobalValues.AMP_SPEED;
    shooter.setShooterVelocity(-rps, -rps-10);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Shooter Within Limit",
        Math.abs(shooter.getKrakenVelocity() - rps) < ShooterGlobalValues.RPM_THRESHOLD);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(shooter.getLeftShooterVelocity() - ShooterGlobalValues.SHOOTER_SPEED) < deadband)
    {
      return true;
    }
    return false;
  }
}
