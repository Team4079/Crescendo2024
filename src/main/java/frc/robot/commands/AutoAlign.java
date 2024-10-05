// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

/** The {@link AutoAlign} command is a command that aligns the robot to the target. */
public class AutoAlign extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Photonvision photonvision;

  /** Rotation PID and offset **/
  private final PIDController rotationalController;

	public AutoAlign(SwerveSubsystem swerveSubsystem, Photonvision limelety) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.photonvision = limelety;
    rotationalController =
        new PIDController(
            BasePIDGlobal.ROTATIONAL_PID.p,
            BasePIDGlobal.ROTATIONAL_PID.i,
            BasePIDGlobal.ROTATIONAL_PID.d);
    rotationalController.setTolerance(2);
    addRequirements(swerveSubsystem, photonvision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // No specific action needed when the command ends.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Horizontal PID and offset
    double horizontalError = -photonvision.getYaw() - 1;
    System.out.println(horizontalError);
    if (Math.abs(horizontalError) >= SwerveGlobalValues.LIMELIGHT_DEADBAND) {
      swerveSubsystem.setDriveSpeeds(0, 0, rotationalController.calculate(horizontalError, 1), false);
    } else {
      swerveSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
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
