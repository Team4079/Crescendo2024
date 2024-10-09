// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
// import frc.robot.utils.GlobalsValues.ShooterConstants;
// import frc.robot.utils.GlobalsValues;
// import frc.robot.utils.GlobalsValues.ShooterGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

public class StagePass extends Command {

  private double deadband;

  private Shooter shooter;

  private double rps;

  /** Creates a new Shoot. */
  public StagePass(Shooter shooter) {
    deadband = 5;
    this.shooter = shooter;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.stopAllMotors();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterVelocity(
        ShooterGlobalValues.STAGE_PASS_RPS, ShooterGlobalValues.STAGE_PASS_RPS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
