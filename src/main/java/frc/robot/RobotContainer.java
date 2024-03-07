
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoAlign;
import frc.robot.commands.TeleOpAlign;
import frc.robot.commands.ShootingSequence;
import frc.robot.commands.PadDrive;
import frc.robot.commands.ShooterRampUp;
import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Jevois;
// import frc.robot.commands.TargetLED;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.Constants;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.Constants.SwerveConstants;

import com.pathplanner.lib.auto.NamedCommands;

// import java.util.HashMap;
// import java.util.function.Consumer;

// import javax.management.InstanceAlreadyExistsException;
// import javax.naming.OperationNotSupportedException;
// import javax.net.ssl.SSLSocket;

// import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;

// import edu.wpi.first.math.kinematics.SwerveModuleState;

//import frc.robot.subsystems.NavX;
// import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.utils.LogitechGamingPad;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.utils.Constants;

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

  // private final SendableChooser<Command> autoChooser;
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

    NamedCommands.registerCommand("autoAlign", new AutoAlign(swerveSubsystem, limelety, led));
    // NamedCommands.registerCommand("FullShoot",
    // new ShootingSequence(swerveSubsystem, limelety, led, pivotyboi, shootyboi));

    swerveSubsystem
        .setDefaultCommand(new PadDrive(swerveSubsystem, pad, opPad, SwerveConstants.isFieldOriented, limelety, led, pivotyboi, shootyboi, intakeyboi));

    // Configure auto chooser
    configureBindings();
    // autoChooser = AutoBuilder.buildAutoChooser("New Auto");
    // SmartDashboard.putData("Auto Chooser", autoChooser);
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
    padX.whileTrue(new TeleOpAlign(swerveSubsystem, limelety, led, pad));
    
    // opPadB.onTrue(new InstantCommand(shootyboi::toggleShooterVelocity));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");
    // return AutoBuilder.followPath(path);

    swerveSubsystem.zeroHeading();
    swerveSubsystem.newPose();
    swerveSubsystem.addRotorPositionsforModules();
    System.out.println(swerveSubsystem.getPose());

    // MUST USE PRESET STARTING POSE; SET TO SAME AS WHERE PATH STARTS
    return new PathPlannerAuto("Full Auto");
  }

  /**
   * Gets the test command
   *
   * @return the command to run in test initial
   *         //
   */
}