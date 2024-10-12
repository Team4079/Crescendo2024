// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;
import frc.robot.utils.GlobalsValues.IntakeGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;
import frc.robot.utils.LogitechGamingPad;

/** Command to control the spinning of the intake mechanism. */
public class SpinIntake extends Command {
  private final Intake intake;
  private final Shooter shooter;
  private final Photonvision photonvision;
  private final LED led;
  private final LogitechGamingPad pad;
  private final Timer timer;
  private final Timer limelightTimer;
  private boolean shouldSpin;

  /**
   * Constructs a new SpinIntake command.
   *
   * @param intake the intake subsystem
   * @param shooter the shooter subsystem
   * @param pad the gamepad controller
   * @param photonvision the limelight subsystem
   * @param led the LED subsystem
   */
  public SpinIntake(
      Intake intake, Shooter shooter, LogitechGamingPad pad, Photonvision photonvision, LED led) {
    this.intake = intake;
    this.shooter = shooter;
    this.pad = pad;
    this.photonvision = photonvision;
    this.led = led;
    timer = new Timer();
    limelightTimer = new Timer();
    addRequirements(intake, led);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    shouldSpin = false;
    shooter.stopAllMotors();
  }

  /**
   * Executes the SpinIntake command.
   *
   * <p>This method is called periodically while the command is scheduled. It updates the
   * SmartDashboard with the current state of `shouldSpin`, toggles the `shouldSpin` state based on
   * the gamepad input, controls the LED state, and manages the intake and shooter mechanisms based
   * on the current conditions.
   */
  @Override
  public void execute() {
    SmartDashboard.putBoolean("should spin", shouldSpin);
    if (pad.getXReleased()) {
      shouldSpin = !shouldSpin;
    }

    ledControl();

    if (!ShooterGlobalValues.HAS_PIECE && shouldSpin) {
      startIntakeAndShooter();
    } else {
      stopIntakeAndShooter();
      handleTimers();
    }
  }

  /**
   * Starts the intake and shooter mechanisms.
   *
   * <p>This method sets the intake velocity to the predefined intake speed and the shooter velocity
   * to the passthrough rate per second. It also resets the timers for the intake and limelight.
   */
  private void startIntakeAndShooter() {
    intake.setIntakeVelocity(IntakeGlobalValues.INTAKE_SPEED);
    shooter.setKrakenVelocity(ShooterGlobalValues.PASSTHROUGH_RPS);
    timer.reset();
    limelightTimer.reset();
  }

  /**
   * Stops the intake and shooter mechanisms.
   *
   * <p>This method stops the kraken motors for both the shooter and intake subsystems.
   */
  private void stopIntakeAndShooter() {
    shooter.stopKraken();
    intake.stopKraken();
  }

  /**
   * Handles the timing logic for the shooter and limelight.
   *
   * <p>This method starts the timers if specific gamepad buttons are not released, manages the
   * shooter timing, controls the limelight flashing, and stops the kraken motors and timers.
   */
  private void handleTimers() {
    if (!pad.getBReleased() && !pad.getRightBumperReleased()) {
      timer.start();
      limelightTimer.start();
      handleShooterTiming();
    }

    shooter.stopKraken();
    intake.stopKraken();
    timer.stop();
  }

  /**
   * Manages the timing for the shooter mechanism.
   *
   * <p>This method sets the shooter velocity to the passthrough rate per second for the first 0.3
   * seconds and then sets it to a velocity of 20 for the next 0.15 seconds.
   */
  private void handleShooterTiming() {
    while (timer.get() < 0.3) {
      shooter.setKrakenVelocity(ShooterGlobalValues.PASSTHROUGH_RPS);
    }

    while (timer.get() < 0.45) {
      shooter.setKrakenVelocity(20);
    }
  }

  /**
   * Controls the LED state based on the shooter and intake status.
   *
   * <p>This method sets the LED color based on the ring sensor status of the shooter and the intake
   * status.
   */
  private void ledControl() {
    if (!shooter.getRingSensor() && intake.getIntakeStatus()) {
      led.setTan();
    } else if (!shooter.getRingSensor()) {
      led.setRed();
    } else if (shooter.getRingSensor() && Math.abs(photonvision.getYaw()) == 0) {
      led.setHighTide();
    } else {
      led.setGreen();
    }
  }

  /**
   * Called once the command ends or is interrupted.
   *
   * @param interrupted whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    intake.stopKraken();
  }

  /**
   * Returns true when the command should end.
   *
   * @return false as this command never ends on its own
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
