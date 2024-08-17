// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.IntakeGlobalValues;
// import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

public class SpinIntake extends Command {
  /** Creates a new SpinIntake. */
  private Intake intake;
  private Shooter shooter;
  private Limelight limelight;
  private LED led;
  private LogitechGamingPad pad;
  private Timer timer;
  private Timer limelightTimer;
  private boolean shouldSpin;
  private boolean d;

  // lIMELIGHT IN THE CONSTRUCOT
  public SpinIntake(Intake intake, Shooter shooter, LogitechGamingPad pad, Limelight limelight, LED led) {
    this.intake = intake;
    this.shooter = shooter;
    this.pad = pad;
    this.limelight = limelight;
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
    d = false;
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    // pad.setRumble(RumbleType.kBothRumble, 1);
    SmartDashboard.putBoolean("intake", d);
    SmartDashboard.putBoolean("should spin", shouldSpin);
    if (pad.getXReleased()) {
      shouldSpin = !shouldSpin;
    }

    if (!shooter.getRingSensor() && intake.getIntakeStatus()) {
      led.setTanColor();
    } else if (!shooter.getRingSensor()) {
      led.setRedColor();
    } else if (shooter.getRingSensor() && Math.abs(limelight.getTx()) == 0) {
      led.setHighTide();
    } else {
      led.setGreenColor();
    }

    if (!ShooterGlobalValues.HAS_PIECE && shouldSpin) {
      intake.setIntakeVelocity(IntakeGlobalValues.INTAKE_SPEED);
      shooter.setKrakenVelocity(ShooterGlobalValues.PASSTHROUGH_RPS);
      // System.out.println("jsd;lfakj;ldsakjf;lsajdf;lkajsd;lkfsa");
      d = true;
      timer.reset();
      limelightTimer.reset();
    } else {

      shooter.stopKraken();
      intake.stopKraken();
      if (!pad.getBReleased() && !pad.getRightBumperReleased()) {
        d = false;
        timer.start();
        limelightTimer.start();
        while (timer.get() < 0.3) {
          // shooter.setShooterVelocity(6, 6);
          shooter.setKrakenVelocity(ShooterGlobalValues.PASSTHROUGH_RPS);
        }

        while (timer.get() < 0.45)
          shooter.setKrakenVelocity(20);
      }

      if (limelightTimer.get() < 3) {
        limelight.flash();
      } else {
        limelight.unflash();
      }

      shooter.stopKraken();
      intake.stopKraken();
      timer.stop();
      timer.reset();

      if (timer.get() > 0.3)
      {
        shooter.setShooterVelocity(-3, -3);
      }
    }

  }

  /** Called once the command ends or is interrupted. */
  @Override
  public void end(boolean interrupted) {
    intake.stopKraken();
  }

  /** Returns true when the command should end. */
  @Override
  public boolean isFinished() {
    return false;
  }
}
