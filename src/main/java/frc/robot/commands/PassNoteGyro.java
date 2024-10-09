// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

public class PassNoteGyro extends Command {
  private final SwerveSubsystem swerve;
  private final Pivot pivot;
  private final Shooter shooter;
  private final boolean isBlueSide;
  private final double angle;
  private final PIDController pidController;

  /** Creates a new PassNoteGyro. */
  public PassNoteGyro(SwerveSubsystem swerve, Pivot pivot, Shooter shooter) {
    this.swerve = swerve;
    this.pivot = pivot;
    this.shooter = shooter;

    isBlueSide =
        DriverStation.getAlliance()
            .orElse(DriverStation.Alliance.Blue)
            .equals(DriverStation.Alliance.Blue);

    if (isBlueSide) {
      angle = ShooterGlobalValues.blueSideAngle;
    } else {
      angle = ShooterGlobalValues.redSideAngle;
    }

    pidController =
        new PIDController(
            BasePIDGlobal.PASS_ROTATIONAL_PID.p,
            BasePIDGlobal.PASS_ROTATIONAL_PID.i,
            BasePIDGlobal.PASS_ROTATIONAL_PID.d);
    pidController.setTolerance(10);
    pidController.enableContinuousInput(0, 360);

    addRequirements(swerve, pivot, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.calculate(swerve.getHeading(), angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Positional Error", pidController.getPositionError());
    if (Math.abs(pidController.getPositionError()) < 10) {
      swerve.stop();
    } else {
      swerve.setDriveSpeeds(0, 0, pidController.calculate(-swerve.getHeading(), angle), false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pidController.getPositionError()) < 10;
  }
}
