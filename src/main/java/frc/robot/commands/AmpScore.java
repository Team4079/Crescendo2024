// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends SequentialCommandGroup {
  public AmpScore(Shooter shooter, Pivot pivot, Limelight limelight, Elevator elevator) {
    addRequirements(shooter, pivot, limelight, elevator);
    addCommands(
        new ParallelCommandGroup(
            new ElevatorAmpSetup(elevator).withTimeout(1.5),
            new SetPivot(pivot, PivotGlobalValues.PIVOT_AMP_ANGLE).withTimeout(1.5),
            new AmpRampUp(shooter).withTimeout(1.5)),
        new PushRingAmp(shooter, limelight).withTimeout(1),
        new StopShooter(shooter).withTimeout(0.05),
        new SetPivot(pivot, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE).withTimeout(0.5),
        new ElevatorRampDown(elevator));
  }
}
