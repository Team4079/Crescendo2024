// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;

public class PivotAlign extends Command {

  // private Pivot pivot;
  // // Get distance when after we mount the limelight
  // private Limelight limelety;
  // private double[] llValues;
  // private double setPoint;
  // private double deadband;
  // private double timeout;

  /** Creates a new Shoot. */
  public PivotAlign(Pivot pivot, Limelight limelety) {
    // this.limelety = limelety;
    // this.pivot = pivot;
    // deadband = 0.5;
    // timeout = 0;
    // addRequirements(pivot, limelety);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // llValues = limelety.getRobotPose_TargetSpace2D();
    // SmartDashboard.putNumber(getName(), 69);

    // change later
  //   setPoint = pivot.shootPos(llValues[2]);

  //   pivot.setPosition(setPoint, setPoint);

  //   if (Math.abs(setPoint - pivot.getPivotPos()) < deadband)
  //   {
  //     timeout++;
  //   }
  //   else
  //   {
  //     timeout = 0;
  //   }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (timeout == 10)
    // {
    //   return true;
    // }
    

    
    return false;
  }
}
