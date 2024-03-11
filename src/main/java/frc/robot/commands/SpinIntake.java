// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.IntakeGlobalValues;
// import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

public class SpinIntake extends Command {
  /** Creates a new SpinIntake. */
  private Intake intake;
  private LogitechGamingPad opPad;
  private boolean shouldSpin;

  public SpinIntake(Intake intake, LogitechGamingPad opPad) {
    this.intake = intake;
    this.opPad = opPad;

    addRequirements(intake);
  }

  /** Called when the command is initially scheduled. */
  @Override
  public void initialize() {
    shouldSpin = true;
  }

  /** Called every time the scheduler runs while the command is scheduled. */
  @Override
  public void execute() {
    if (opPad.getXReleased()) {
      shouldSpin = !shouldSpin;
    }

    if (shouldSpin && !ShooterGlobalValues.HAS_PIECE) {
      intake.setIntakeVelocity(IntakeGlobalValues.INTAKE_SPEED);
    } else {
      intake.stopKraken();
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
