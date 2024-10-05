// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.utils.GlobalsValues.ElevatorGlobalValues;

public class ElevatorRampDown extends Command {
  Elevator elevator;
  /** Creates a new ElevatorAmpSetup. */
  public ElevatorRampDown(Elevator elevator) {
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // elevator.setState(ElevatorState.DOWN);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(elevator.getElevatorPosition() - ElevatorGlobalValues.ELEVATOR_DOWN) < 0.5;
    return false;
  }
}
