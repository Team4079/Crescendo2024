// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.ShooterGlobalValues;

/**
 * The AmpRampUpAlt command ramps up the shooter to a specified speed.
 * It uses the Shooter subsystem to control the shooter motors.
 */
public class AmpRampUpAlt extends Command {
	private final double deadband;
	private final Shooter shooter;

	/**
	 * Creates a new AmpRampUpAlt command.
	 *
	 * @param shooter The Shooter subsystem used by this command.
	 */
	public AmpRampUpAlt(Shooter shooter) {
		deadband = 5;
		this.shooter = shooter;
		addRequirements(shooter);
	}

	/**
	 * Called when the command is initially scheduled.
	 * Sets the shooter velocity to the specified AMP speed.
	 */
	@Override
	public void initialize() {
		shooter.setShooterVelocity(-ShooterGlobalValues.AMP_SPEED, -ShooterGlobalValues.AMP_SPEED);
	}

	/**
	 * Called every time the scheduler runs while the command is scheduled.
	 * Updates the SmartDashboard with the shooter's velocity status.
	 */
	@Override
	public void execute() {
		SmartDashboard.putBoolean("Shooter Within Limit", Math.abs(shooter.getKrakenVelocity()) < ShooterGlobalValues.RPM_THRESHOLD); // rps???
	}

	/**
	 * Called once the command ends or is interrupted.
	 *
	 * @param interrupted Whether the command was interrupted/canceled.
	 */
	@Override
	public void end(boolean interrupted) {
		// No specific action needed when the command ends.
	}

	/**
	 * Returns true when the command should end.
	 * The command ends when the shooter's velocity is within the deadband of the target speed.
	 *
	 * @return true if the shooter's velocity is within the deadband, false otherwise.
	 */
	@Override
	public boolean isFinished() {
		return Math.abs(shooter.getLeftShooterVelocity() - ShooterGlobalValues.SHOOTER_SPEED) < deadband;
	}
}
