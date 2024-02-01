// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import java.lang.annotation.Target;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.LED;
// import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.utils.PID;

// @SuppressWarnings("unused")
// public class TargetLED extends Command {
//   /** Creates a new AutoAlign. */
//   private Limelight limelight;
//   private SwerveSubsystem swerveSubsystem;
//   private LED led;
  
//   private PID horizontalPID;
//   private double horizontalError;

//   private PID verticalPID;
//   private double verticalError;

//   private PID rotationalPID;
//   private double rotationalError;
//   private double printSlow = 0;

//   private double limelightDeadband = 5.0;

//   public TargetLED(Limelight limelight, LED led, SwerveSubsystem swerveSubsystem) {
//     // Use addRequirements() here to declare subsystem dependencies.''    this.limelight = limelight;
//     this.limelight = limelight;
//     this.led = led;
//     this.swerveSubsystem = swerveSubsystem;

//     horizontalPID = new PID(0.05, 0.075, 0.03, 0);
//     verticalPID = new PID(0.24, 0.0085, 0.03, 0);
//     rotationalPID = new PID(0.05, 0.007, 0.01, 0);
//     addRequirements(limelight, led, swerveSubsystem);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (limelight.isTarget()) {
//       if (Math.abs(horizontalError) >= 5) {
//         led.rainbow(60, 255, 255); // Set led to green
//       } else {
//         led.rainbow(20, 255, 255); // Set led to orange
//       }
//       swerveSubsystem.addVision(limelight.getRobotPosition());
//     } else {
//       led.rainbow(0, 255, 255); // Set led to red
//     }

//     // rotationalError = limelight.getRobotPose_TargetSpace2D().getRotation().getDegrees(); // Only runs when detects an AprilTag
//     rotationalError = -limelight.getRobotPose_TargetSpace2D()[4];// limelight.getTs();
//     horizontalError = -limelight.getTx();

//     if (printSlow == 100)
//     {
//       System.out.println(horizontalError);

//       // Rotational Value = index of 3 (value 4)
//       printSlow = 0;
//     }
//     else
//     {
//       printSlow += 1;
//     }
//     // verticalError = -limelight.getTy();
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }