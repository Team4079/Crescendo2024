// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.SwerveConstants;
import frc.robot.utils.Constants.SwerveConstants.BasePIDConstants;
import frc.robot.utils.PID;

@SuppressWarnings("unused")
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  private Limelight limelight;
  private SwerveSubsystem swerveSubsystem;
  private LED led;
  
  // Horizontal PID and offset
  private PID horizontalPID;
  private double horizontalError;

  // Vertical PID and offset
  private PID verticalPID;
  private double verticalError;

  // Rotation PID and offset
  private PID rotationalPID;
  private double rotationalError;

  private double printSlow = 0;

  public AutoAlign(SwerveSubsystem swerveSubsystem, Limelight limelight, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    this.led = led;
    horizontalPID = BasePIDConstants.horizontalPID;
    verticalPID = BasePIDConstants.verticalPID;
    rotationalPID = BasePIDConstants.rotationalPID;
    addRequirements(swerveSubsystem, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // rotationalError = limelight.getRobotPose_TargetSpace2D().getRotation().getDegrees(); // Only runs when detects an AprilTag
    rotationalError = -limelight.getRobotPose_TargetSpace2D()[4];// limelight.getTs();
    horizontalError = -limelight.getTx();

    if (printSlow == 100)
    {
      System.out.println(horizontalError);

      // Rotational Value = index of 3 (value 4)
      printSlow = 0;
    }
    else
    {
      printSlow += 1;
    }
    // verticalError = -limelight.getTy();

    if (Math.abs(horizontalError) >= SwerveConstants.limelightDeadband)
    {
      swerveSubsystem.drive(0, 0, rotationalPID.calculate(horizontalError, 0), false);
    }
    else
    {
      swerveSubsystem.stopModules();
    }

    
    // Vision LED
    if (limelight.isTarget()) {
      if (Math.abs(horizontalError) <= SwerveConstants.limelightDeadband) {
        led.rainbow(SwerveConstants.greenLED[0], SwerveConstants.greenLED[1], SwerveConstants.greenLED[2]); // Set led to green
      } else {
        led.rainbow(SwerveConstants.orangeLED[0], SwerveConstants.orangeLED[1], SwerveConstants.orangeLED[2]); // Set led to orange
      }
      swerveSubsystem.addVision(limelight.getRobotPosition());
    } else {
      // Remove Red LED light when in competition.
      led.rainbow(SwerveConstants.redLED[0], SwerveConstants.redLED[1], SwerveConstants.redLED[2]); // Set led to red
    }

    
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