// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.sampled.LineEvent;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;
import frc.robot.utils.PID;

public class PivotShooterSetUp extends Command {
  private Pivot pivot;
  private Shooter shooter;
  private Limelight limelight;
  private Timer timer;
  private double deadband;
  private boolean isDone;
  private double pos;
  private double velocity;
  private double rps;

  private SwerveSubsystem swerveSubsystem;

  // Horizontal PID and offset
  private double horizontalError;

  // Rotation PID and offset
  private PIDController rotationalController;
  private PIDController velocityPIDController;

  private double timeout = 0;
  private boolean end = false;
  /** Creates a new PivotShooterSetUp. */
  public PivotShooterSetUp(Pivot pivot, Shooter shooter, Limelight limelight, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot, shooter, limelight, swerveSubsystem);
    this.swerveSubsystem = swerveSubsystem;
    timer = new Timer();
    rotationalController = new PIDController(BasePIDGlobal.rotationalPID.p, BasePIDGlobal.rotationalPID.i, BasePIDGlobal.rotationalPID.d);
    velocityPIDController = new PIDController(0.037, 0, 0.000005);
    rotationalController.setTolerance(3);
    this.pivot = pivot;
    this.limelight = limelight;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    

    deadband = 0.1;
    isDone = false;
    if (limelight.getDistance() < 1.1) {
      pos = PivotGlobalValues.PIVOT_SUBWOOFER_ANGLE;
    }
    else {
      pos = limelight.getPivotPosition();
    }

    rps = ShooterGlobalValues.SHOOTER_SPEED + (limelight.getDistance()-1.5) * 5;
    // shooter.setShooterVelocity(-rps, -rps);\[]
    shooter.setShooterVelocity(-rps, -rps);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    velocity = velocityPIDController.calculate(pivot.getPivotLeftPos(), pos);
    SmartDashboard.putNumber("Error Pivot", -pivot.getPivotLeftPos() + pos);
    SmartDashboard.putNumber("Setpoint", pos);
    SmartDashboard.putNumber("Velocity Pivot", velocity);

    horizontalError = -limelight.getTx();
    System.out.println(horizontalError);
    if (Math.abs(horizontalError) >= SwerveGlobalValues.limelightDeadband) {
      swerveSubsystem.drive(0, 0, rotationalController.calculate(horizontalError, 0), false);
    } else {
      swerveSubsystem.stopModules();
      timeout++;
    }

    if (Math.abs(pivot.getPivotLeftPos() - pos) < deadband)
    {
       pivot.stopMotors();
    }
    else {
      pivot.movePivot(velocity);
    }
    
    if (Math.abs(pivot.getPivotLeftPos() - pos) <= deadband)
    {
      timer.start();
      if (timer.get() >= 0.1) {
        isDone = true;
      }
    }
    
    else {
      timer.reset();
      isDone = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.stopMotors();
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}