// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.crypto.spec.RC2ParameterSpec;
import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.LimelightGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;
import frc.robot.utils.PID;

/** The {@link AutoAlign} command is a command that aligns the robot to the target. */
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  private SwerveSubsystem swerveSubsystem;
  private Limelight limelight;

  // Horizontal PID and offset
  private double horizontalError;

  // Rotation PID and offset
  private PIDController rotationalController;

  private double timeout = 0;

  public AutoAlign(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    rotationalController = new PIDController(BasePIDGlobal.ROTATIONAL_PID.p, BasePIDGlobal.ROTATIONAL_PID.i, BasePIDGlobal.ROTATIONAL_PID.d);
    rotationalController.setTolerance(3);
    addRequirements(swerveSubsystem, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    horizontalError = -limelight.getTx();
    System.out.println(horizontalError);
    if (Math.abs(horizontalError) >= SwerveGlobalValues.LIMELIGHT_DEADBAND) {
      swerveSubsystem.drive(0, 0, rotationalController.calculate(horizontalError, 0), false);
    } else {
      swerveSubsystem.stopModules();
      timeout++;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Checks if the april tag is within the deadband for at least half a second
    // if (horizontalError <= SwerveGlobalValues.limelightDeadband && timeout == 20) {
    //   timeout = 0;
    //   return true;
    // }

    return false;
  }
}