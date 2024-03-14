
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoAlign;
import frc.robot.commands.LimelightValues;
import frc.robot.commands.LowerPivot;
import frc.robot.commands.TeleOpAlign;
import frc.robot.commands.ShootingSequence;
import frc.robot.commands.SpinIntake;
import frc.robot.commands.StartIntake;
import frc.robot.commands.StopIntake;
import frc.robot.commands.PadDrive;
import frc.robot.commands.PadPivot;
import frc.robot.commands.PadShoot;
import frc.robot.commands.SetLED;
import frc.robot.commands.ShooterRampDown;
import frc.robot.commands.ShooterRampUp;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Jevois;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.GlobalsValues;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;

import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

@SuppressWarnings("unused")
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final LogitechGamingPad pad;
  private final LogitechGamingPad opPad;
  private final LED led;
  private final Limelight limelety;
  // private final Jevois jevois;
  private final Pivot pivotyboi;
  private final Shooter shootyboi;
  private final Intake intakeyboi;

  private final JoystickButton padA;
  private final JoystickButton padB;
  private final JoystickButton padX;
  private final JoystickButton padY;
  private final JoystickButton rightBumper;
  private final JoystickButton leftBumper;

  private final JoystickButton opPadA;
  private final JoystickButton opPadB;
  private final JoystickButton opPadX;
  private final JoystickButton opPadY;
  private final JoystickButton opRightBumper;
  private final JoystickButton opLeftBumper;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    pad = new LogitechGamingPad(0);
    opPad = new LogitechGamingPad(1);
    led = new LED();
    limelety = new Limelight();
    // jevois = new Jevois();
    pivotyboi = new Pivot();
    shootyboi = new Shooter();
    intakeyboi = new Intake();

    padA = new JoystickButton(pad, 1);
    padB = new JoystickButton(pad, 2);
    padX = new JoystickButton(pad, 3);
    padY = new JoystickButton(pad, 4);
    rightBumper = new JoystickButton(pad, 6);
    leftBumper = new JoystickButton(pad, 5);

    opPadA = new JoystickButton(opPad, 1);
    opPadB = new JoystickButton(opPad, 2);
    opPadX = new JoystickButton(opPad, 3);
    opPadY = new JoystickButton(opPad, 4);
    opRightBumper = new JoystickButton(opPad, 6);
    opLeftBumper = new JoystickButton(opPad, 5);

    swerveSubsystem = new SwerveSubsystem();

    // index from 0
    // 0 is left-right distance from tag (left is +, right is -, accurate to +- 5cm
    // per meter)
    // 1
    // 2 is forward-backward distance from tag (forward is +, backward is -,
    // accurate to +- 5cm per meter)
    // 3
    // 4 is rotation (clockwise is -) (accurate to +-0.5 a degree)
    // 5

    NamedCommands.registerCommand("autoAlign", new AutoAlign(swerveSubsystem));
    NamedCommands.registerCommand("startIntake", new StartIntake(intakeyboi));
    NamedCommands.registerCommand("stopIntake", new StopIntake(intakeyboi));
    swerveSubsystem
        .setDefaultCommand(new PadDrive(swerveSubsystem, pad, SwerveGlobalValues.isFieldOriented));

    intakeyboi.setDefaultCommand(new SpinIntake(intakeyboi, opPad));
    pivotyboi.setDefaultCommand(new PadPivot(pivotyboi, opPad));
    limelety.setDefaultCommand(new LimelightValues(limelety));
    led.setDefaultCommand(new SetLED(led));
    shootyboi.setDefaultCommand(new PadShoot(shootyboi, opPad));

    // Configure auto chooser
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    padA.onTrue(new InstantCommand(swerveSubsystem::addRotorPositionsforModules));
    padB.onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
    padY.onTrue(new InstantCommand(swerveSubsystem::newPose));
    padX.whileTrue(new TeleOpAlign(swerveSubsystem, pad));

    opPadB.onTrue(new ShooterRampUp(shootyboi));
    opPadA.onTrue(new ShooterRampDown(shootyboi));
    // Shoot command for Ria
    opPadY.onTrue(new InstantCommand(shootyboi::toggleShooterVelocity));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @param void
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    swerveSubsystem.zeroHeading();
    swerveSubsystem.newPose();
    swerveSubsystem.addRotorPositionsforModules();
    System.out.println(swerveSubsystem.getPose());

    // MUST USE PRESET STARTING POSE; SET TO SAME AS WHERE PATH STARTS
    return new PathPlannerAuto("Center Auto");
  }

}