// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.amp;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.utils.GlobalsValues.ElevatorGlobalValues;

public class ElevatorRampUp extends Command {
  Elevator elevator;

  /** Creates a new ElevatorAmpSetup. */
  public ElevatorRampUp(Elevator elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setState(ElevatorState.UP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(elevator.getElevatorPosition() - ElevatorGlobalValues.ELEVATOR_UP) < 0.5;
  }
}
