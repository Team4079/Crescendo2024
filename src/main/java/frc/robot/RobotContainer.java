// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.amp.AmpScore;
import frc.robot.commands.amp.ElevatorRampDown;
import frc.robot.commands.amp.ElevatorRampUp;
import frc.robot.commands.auto.ShootRingAuto;
import frc.robot.commands.intake.PulseDown;
import frc.robot.commands.intake.ReverseIntake;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.intake.StartIntake;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.pad.PadDrive;
import frc.robot.commands.pad.PadElevator;
import frc.robot.commands.pad.PadPivot;
import frc.robot.commands.pad.PadShoot;
import frc.robot.commands.speaker.AutoAlign;
import frc.robot.commands.speaker.SubwooferShot;
import frc.robot.subsystems.*;
import frc.robot.utils.GlobalsValues.PivotGlobalValues;
import frc.robot.utils.GlobalsValues.SwerveGlobalValues;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.commands.stage.PassNoteGyro;

import org.json.simple.parser.ParseException;

import java.io.IOException;

import javax.print.attribute.standard.MediaSize.NA;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final LogitechGamingPad pad;
  private final LogitechGamingPad opPad;
  private final LogitechGamingPad commandCheckPad;
  private final Pivot pivotyboi;
  private final Shooter shootyboi;
  private final Intake intakeyboi;
  private final Elevator elevator;
  private final LED led;

  private final JoystickButton padA;
  private final JoystickButton padB;
  private final JoystickButton padX;
  private final JoystickButton padY;
  private final JoystickButton rightBumper;
  private final JoystickButton leftBumper;
  private final JoystickButton startButton;

  private final JoystickButton commandCheckPadA;

  private final JoystickButton opPadA;
  private final JoystickButton opPadB;
  private final JoystickButton opPadX;
  private final JoystickButton opPadY;
  private final JoystickButton opRightBumper;
  private final JoystickButton opLeftBumper;

  private final Photonvision photonvision;

  // private final Command autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    pad = new LogitechGamingPad(0);
    opPad = new LogitechGamingPad(1);
    commandCheckPad = new LogitechGamingPad(2);
    led = new LED();
    // jevois = new Jevois();
    pivotyboi = new Pivot();
    shootyboi = new Shooter();
    intakeyboi = new Intake();
    photonvision = new Photonvision();

    padA = new JoystickButton(pad, 1);
    padB = new JoystickButton(pad, 2);
    padX = new JoystickButton(pad, 3);
    padY = new JoystickButton(pad, 4);
    rightBumper = new JoystickButton(pad, 6);
    leftBumper = new JoystickButton(pad, 5);
    startButton = new JoystickButton(pad, 8);

    commandCheckPadA = new JoystickButton(commandCheckPad, 1);

    opPadA = new JoystickButton(opPad, 1);
    opPadB = new JoystickButton(opPad, 2);
    opPadX = new JoystickButton(opPad, 3);
    opPadY = new JoystickButton(opPad, 4);
    opRightBumper = new JoystickButton(opPad, 6);
    opLeftBumper = new JoystickButton(opPad, 5);

    swerveSubsystem = new SwerveSubsystem(photonvision);
    elevator = new Elevator();

    // index from 0
    // 0 is left-right distance from tag (left is +, right is -, accurate to +- 5cm
    // per meter)
    // 1
    // 2 is forward-backward distance from tag (forward is +, backward is -,
    // accurate to +- 5cm per meter)
    // 3
    // 4 is rotation (clockwise is -) (accurate to +-0.5 a degree)
    // 5

    // NamedCommands.registerCommand("autoAlign", new AutoAlign(swerveSubsystem));
    NamedCommands.registerCommand(
        "startIntake", new StartIntake(intakeyboi, shootyboi).withTimeout(6));
    NamedCommands.registerCommand("stopIntake", new StopIntake(intakeyboi, shootyboi));
    NamedCommands.registerCommand("pushRing", new PushRing(shootyboi));
    NamedCommands.registerCommand(
        "shootRing", new ShootRing(shootyboi, pivotyboi, swerveSubsystem, photonvision));
    NamedCommands.registerCommand(
        "setPivotDown", new SetPivot(pivotyboi, PivotGlobalValues.PIVOT_NEUTRAL_ANGLE));
    NamedCommands.registerCommand(
        "setPivot", new SetPivot(pivotyboi, PivotGlobalValues.PIVOT_SUBWOOFER_ANGLE));
    NamedCommands.registerCommand(
        "SubwooferShot", new SubwooferShot(shootyboi, pivotyboi, photonvision));
    NamedCommands.registerCommand("pushback", new PulseDown(intakeyboi, shootyboi));
    NamedCommands.registerCommand("autoShootRing", new ShootRingAuto(shootyboi, pivotyboi, swerveSubsystem, photonvision));
    NamedCommands.registerCommand("stop", new InstantCommand(swerveSubsystem::stop));

    swerveSubsystem.setDefaultCommand(
        new PadDrive(swerveSubsystem, pad, SwerveGlobalValues.FIELD_ORIENTATED));
    intakeyboi.setDefaultCommand(new SpinIntake(intakeyboi, shootyboi, pad, photonvision, led));
    pivotyboi.setDefaultCommand(new PadPivot(pivotyboi, pad));
    shootyboi.setDefaultCommand(
        new PadShoot(shootyboi, swerveSubsystem, pad, photonvision, pivotyboi));
    elevator.setDefaultCommand(new PadElevator(elevator, pad));
    configureBindings();


    //autoChooser = AutoBuilder.buildAutoChooser("1MeterForward");
    //SmartDashboard.putData("Auto Chooser", autoChooser);
    // autoChooser = AutoBuilder.buildAuto("1MeterForward");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController
   * 
   * ontroller Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    padA.onTrue(new SubwooferShot(shootyboi, pivotyboi, photonvision));
    // padA.onTrue(new InstantCommand(elevator::setState(ElevatorState.UP)));
    padB.onTrue(new InstantCommand(swerveSubsystem::resetPidgey));
    // x is intake

    // padY.whileTrue(new AutoAlign(swerveSubsystem, photonvision));
    padY.whileTrue(new ReverseIntake(intakeyboi, shootyboi));
    // padY.whileTrue(new PassNoteGyro(swerveSubsystem, pivotyboi, shootyboi));

    opPadA.onTrue(new ElevatorRampUp(elevator));
    opPadB.onTrue(new ElevatorRampDown(elevator));
    opPadX.onTrue(new InstantCommand(swerveSubsystem::setCustomDrivePID));

    commandCheckPadA.onTrue(new InstantCommand());

    rightBumper.onTrue(new ShootRing(shootyboi, pivotyboi, swerveSubsystem, photonvision));
    // rightBumper.whileTrue(new ElevatorRampDown(elevator));
    leftBumper.onTrue(new AmpScore(shootyboi, pivotyboi, photonvision, elevator));
    // startButton.onTrue(new StagePass(shootyboi));
    startButton.onTrue(new PassNoteGyro(swerveSubsystem, pad).withTimeout(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() throws IOException, ParseException {
    swerveSubsystem.resetDrive();
    swerveSubsystem.resetPidgey();
    swerveSubsystem.zeroPose();

    // MUST USE PRESET STARTING POSE; SET TO SAME AS WHERE PATH STARTS

    return new PathPlannerAuto("screw you shawn frfr");
    // return new PathPlannerAuto("screw you om part");


    // return new SubwooferShot(shootyboi, pivotyboi, photonvision); 
    // return new PathPlannerAuto("Note4");
    // return new InstantCommand();

    // return AutoBuilder.followPath(PathPlannerPath.fromPathFile("1MeterForward"));
  }

  public Command setTeleopPID() {
    return new InstantCommand(swerveSubsystem::setTelePID);
  }


  public Command setAutoPID() {
    return new InstantCommand(swerveSubsystem::setAutoPID);
  }

  public Command setRotationPidgey() {
    return new InstantCommand(swerveSubsystem::setHeading);
  }

}