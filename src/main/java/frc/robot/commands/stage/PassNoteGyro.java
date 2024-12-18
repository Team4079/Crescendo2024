// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//SOTOP SEAT STEALINGDAJGIDGAIDJFPA

package frc.robot.commands.stage;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.pad.PadDrive.Coordinate;
import frc.robot.commands.pad.PadDrive.*;

import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.MotorGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

public class PassNoteGyro extends Command {
  private final SwerveSubsystem swerve;
  private final boolean isBlueSide;
  private final double angle;
  private final LogitechGamingPad pad;
  private final PIDController pidController;

  /** Creates a new PassNoteGyro. */
  public PassNoteGyro(SwerveSubsystem swerve, LogitechGamingPad pad) {
    this.swerve = swerve;
    this.pad = pad;
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

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.calculate(-swerve.getHeading(), angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = -pad.getLeftAnalogXAxis() * MotorGlobalValues.MAX_SPEED;
    if (Math.abs(x) < SwerveGlobalValues.xDEADZONE) {
      x = 0;
    } // this code doens't make sense, it's just setting x to 0 if it's less than the deadzone fix it

    double y = -pad.getLeftAnalogYAxis() * MotorGlobalValues.MAX_SPEED;
    if (Math.abs(y) < SwerveGlobalValues.yDEADZONE) {
      y = 0;
    }

    Coordinate cords = new Coordinate(x, y);


    SmartDashboard.putNumber("Positional Error", pidController.getPositionError());
    if (Math.abs(pidController.getPositionError()) < 5) {
      swerve.stop();
    } else {
      swerve.setDriveSpeeds(cords.getY(), cords.getX(), pidController.calculate(-swerve.getHeading(), angle), true);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(pidController.getPositionError()) < 10;
    return false;
  }
}