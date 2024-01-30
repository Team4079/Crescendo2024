// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.PID;

@SuppressWarnings("unused")
public class AutoAlign extends Command {
  /** Creates a new AutoAlign. */
  private Limelight limelight;
  private SwerveSubsystem swerveSubsystem;
  private LED led;
  
  private PID horizontalPID;
  private double horizontalError;

  private PID verticalPID;
  private double verticalError;

  private PID rotationalPID;
  private double[] rotationalError;
  public double printSlow = 0;

  public AutoAlign(SwerveSubsystem swerveSubsystem, Limelight limelight, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerveSubsystem = swerveSubsystem;
    this.limelight = limelight;
    this.led = led;
    horizontalPID = new PID(0.05, 0.007, 0.01, 0);
    verticalPID = new PID(0.24, 0.0085, 0.03, 0);
    rotationalPID = new PID(0.05, 0.007, 0.01, 0);
    addRequirements(swerveSubsystem, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.isTarget()) {
      led.rainbowOn();
      swerveSubsystem.addVision(limelight.getRobotPosition());
    } else {
      led.rainbowOff();
    }

    // rotationalError = limelight.getRobotPose_TargetSpace2D().getRotation().getDegrees(); // Only runs when detects an AprilTag
    rotationalError = limelight.getRobotPose_TargetSpace2D();// limelight.getTs();
    horizontalError = -limelight.getTx();

    if (printSlow == 100)
    {
      for (int i = 0; i < rotationalError.length; i++) {
        System.out.println(i + "th vaule: " + rotationalError[i] + "\n-------------------------------------");
      }

      // Rotational Value = index of 3 (value 4)
      printSlow = 0;
    }
    else
    {
      printSlow += 1;
    }
    // verticalError = -limelight.getTy();
    swerveSubsystem.drive(0, horizontalPID.calculate(horizontalError, 0), rotationalPID.calculate(-rotationalError[4], 0) , false);
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