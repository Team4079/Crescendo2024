// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ShooterConstants;

@SuppressWarnings("unused")
public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX falconOne;
  private TalonFX falconTwo;

  public Shooter() {
    falconOne = new TalonFX(ShooterConstants.FALCON_ONE_ID);
    falconTwo = new TalonFX(ShooterConstants.FALCON_TWO_ID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void start() {

  }

  private void stop() {

  }
}
