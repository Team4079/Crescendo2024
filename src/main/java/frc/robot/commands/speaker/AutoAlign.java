// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.speaker;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

/** The {@link AutoAlign} command is a command that aligns the robot to the target. */
public class AutoAlign extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Photonvision photonvision;

  /** Rotation PID and offset * */
  private final PIDController rotationalController;
  private double measurement_yaw;
  private PhotonCamera camera;

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
    camera = photonvision.getBestCamera();
    SmartDashboard.putString("cam used for align", camera.getName());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Horizontal PID and offset
    measurement_yaw = photonvision.getYaw(camera);
    SmartDashboard.putNumber("alignment error", rotationalController.getPositionError());
    SmartDashboard.putNumber("alignment setpoint", rotationalController.getSetpoint());
    SmartDashboard.putString("Camera Used", camera.getName());
    
    System.out.println(measurement_yaw);
    if (Math.abs(measurement_yaw) >= SwerveGlobalValues.LIMELIGHT_DEADBAND) {
      swerveSubsystem.setDriveSpeeds(
          0, 0, rotationalController.calculate(measurement_yaw, photonvision.getOffset(camera)), false);
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
    return photonvision.getYaw(camera) == 4079;
  }
}