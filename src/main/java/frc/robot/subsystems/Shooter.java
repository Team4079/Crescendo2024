// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Hell.ShooterConstants;

@SuppressWarnings("unused")
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX leftFalcon;
  private TalonFX rightFalcon;

  private TalonFX oneKaren;

  public Shooter() {
    leftFalcon = new TalonFX(ShooterConstants.FALCON_LEFT_ID);
    rightFalcon = new TalonFX(ShooterConstants.FALCON_RIGHT_ID);

    oneKaren = new TalonFX(ShooterConstants.KAREN_ONE_ID);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setVelocity(){
    
  }

  private void stop() {

  }
}
