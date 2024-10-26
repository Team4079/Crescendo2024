// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.speaker;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Photonvision;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues.BasePIDGlobal;

import java.util.logging.LogManager;

import org.photonvision.PhotonCamera;

/** The {@link AutoAlign} command is a command that aligns the robot to the target. */
public class AutoAlign extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Photonvision photonvision;

  /** Rotation PID and offset * */
  private final PIDController rotationalController;

  private double measurement_yaw;

  public AutoAlign(SwerveSubsystem swerveSubsystem, Photonvision photonvision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.photonvision = photonvision;
    // measurement_yaw = photonvision.getSubwooferYaw();
    measurement_yaw = photonvision.getYaw();
    rotationalController =
        new PIDController(
            BasePIDGlobal.ROTATIONAL_PID.p,
            BasePIDGlobal.ROTATIONAL_PID.i,
            BasePIDGlobal.ROTATIONAL_PID.d);
    rotationalController.setTolerance(1.5);
    rotationalController.setSetpoint(0);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // measurement_yaw = photonvision.getSubwooferYaw();
    measurement_yaw = photonvision.getYaw();
    // Horizontal PID and offset
    // SmartDashboard.putNumber("alignment error", rotationalController.getPositionError());
    // SmartDashboard.putNumber("alignment setpoint", rotationalController.getSetpoint());
    // SmartDashboard.putBoolean("Robot Aligned", rotationalController.atSetpoint());
    // SmartDashboard.putNumber("measurement yaw", photonvision.getSubwooferYaw());

    System.out.println(measurement_yaw);

    double error = measurement_yaw;

    SmartDashboard.putNumber("align error", error);
    SmartDashboard.putNumber("align yaw", measurement_yaw);



    if (photonvision.hasTag()) {
      // if (measurement_yaw > 0) {
        swerveSubsystem.setDriveSpeeds(0, 0, rotationalController.calculate(error, 0), false);
    }

    else{
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
    // return photonvision.getYaw(camera) == 4079;
    return false;
  }
}