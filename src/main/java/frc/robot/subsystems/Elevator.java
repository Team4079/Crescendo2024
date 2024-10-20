package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues.ElevatorGlobalValues;

/**
 * The Elevator subsystem controls the elevator mechanism of the robot. It uses a CANSparkMax motor
 * controller and a PID controller to manage the elevator's position.
 */
public class Elevator extends SubsystemBase {

  /** Enum representing the possible states of the elevator. */
  public enum ElevatorState {
    UP,
    DOWN,
    CLIMB
  }

  // The motor controller for the elevator.
  private final CANSparkMax elevatorMotorSparkMax =
      new CANSparkMax(ElevatorGlobalValues.ELEVATOR_NEO_ID, MotorType.kBrushless);

  private final CANSparkMax passMotorSparkMax =
      new CANSparkMax(ElevatorGlobalValues.PASS_NEO_ID, MotorType.kBrushless);

  // The current state of the elevator.
  private ElevatorState state = ElevatorState.DOWN;

  /**
   * Constructor for the Elevator subsystem. Initializes the motor controller, sets default
   * configurations, and logs initial data.
   */
  public Elevator() {
    elevatorMotorSparkMax.restoreFactoryDefaults();
    elevatorMotorSparkMax.setIdleMode(IdleMode.kBrake);
    elevatorMotorSparkMax.setInverted(true);
    elevatorMotorSparkMax.setClosedLoopRampRate(ElevatorGlobalValues.closedLoopRampRate);
    getEncoder().setPosition(0.25);

    elevatorMotorSparkMax.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, ElevatorGlobalValues.SOFTLIMIT_FOWARD);
    elevatorMotorSparkMax.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, ElevatorGlobalValues.SOFTLIMIT_REVERSE);

    elevatorMotorSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);

    elevatorMotorSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    passMotorSparkMax.restoreFactoryDefaults();
    passMotorSparkMax.setIdleMode(IdleMode.kBrake);

    passMotorSparkMax.setClosedLoopRampRate(ElevatorGlobalValues.passClosedLoopRampRate);

    getPIDController().setP(ElevatorGlobalValues.kP);
    getPIDController().setI(ElevatorGlobalValues.kI);
    getPIDController().setD(ElevatorGlobalValues.kD);
    getPIDController().setIZone(ElevatorGlobalValues.kIz);
    getPIDController().setFF(ElevatorGlobalValues.kFF);
    getPIDController()
        .setOutputRange(ElevatorGlobalValues.kMinOutput, ElevatorGlobalValues.kMaxOutput);

    getPIDController().setSmartMotionAllowedClosedLoopError(0.0987, 0);
    getPIDController().setSmartMotionMaxAccel(10500, 0);
    getPIDController().setSmartMotionMaxVelocity(11000, 0);

    getPassPIDController().setP(ElevatorGlobalValues.PasskP);
    getPassPIDController().setI(ElevatorGlobalValues.PasskI);
    getPassPIDController().setD(ElevatorGlobalValues.PasskD);

    logData();
  }

  /**
   * This method is called periodically to update the elevator position based on its current state.
   * If the state is UP, it sets the elevator position to the up position defined in
   * ElevatorGlobalValues. If the state is DOWN, it sets the elevator position to the down position
   * defined in ElevatorGlobalValues.
   */
  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Elevator Output", elevatorMotorSparkMax.getAppliedOutput());

    if (!ElevatorGlobalValues.ELEVATOR_JOYSTICKS) {
      if (state == ElevatorState.UP) {
        setElevatorPosition(ElevatorGlobalValues.ELEVATOR_UP);
        setPassSpeed(0.5);

      } else if (state == ElevatorState.DOWN) {
        setElevatorPosition(ElevatorGlobalValues.ELEVATOR_DOWN);
        stopPassMotor();
      }

      else if (state == ElevatorState.CLIMB)
      {
        setElevatorPosition(ElevatorGlobalValues.ELEVATOR_CLIMB);
        stopPassMotor();
      }
    }

    // if (Math.abs(elevatorMotorSparkMax.getAppliedOutput()) < 0.01
    //     && ElevatorGlobalValues.ELEVATOR_JOYSTICKS) {
    //   moveElevator(0.1);
    // }

    // logData();
  }

  public void setPassSpeed(double speed) {
    passMotorSparkMax.set(speed);
  }

  public void stopPassMotor() {
    passMotorSparkMax.set(0);
  }

  /**
   * Sets the state of the elevator.
   *
   * @param state The state to set the elevator to.
   */
  public void setState(ElevatorState state) {
    this.state = state;
  }

  /**
   * Sets the position of the elevator.
   *
   * @param position The position to set the elevator to.
   */
  public void setElevatorPosition(double position) {
    elevatorMotorSparkMax.getPIDController().setReference(position, ControlType.kSmartMotion);
  }

  /**
   * Gets the current position of the elevator.
   *
   * @return The current position of the elevator.
   */
  public double getElevatorPosition() {
    return elevatorMotorSparkMax.getEncoder().getPosition();
  }

  /**
   * Gets the PID controller for the elevator's motor.
   *
   * @return The PID controller for the elevator's motor.
   */
  public SparkPIDController getPIDController() {
    return elevatorMotorSparkMax.getPIDController();
  }

  public SparkPIDController getPassPIDController() {
    return passMotorSparkMax.getPIDController();
  }

  /**
   * Gets the encoder for the elevator's motor.
   *
   * @return The encoder for the elevator's motor.
   */
  public RelativeEncoder getEncoder() {
    return elevatorMotorSparkMax.getEncoder();
  }

  /**
   * Moves the elevator at a specified speed.
   *
   * @param speed The speed to move the elevator at.
   */
  public void moveElevator(double speed) {
    SmartDashboard.putNumber("Elevator Speed", speed);
    elevatorMotorSparkMax.set(speed);
  }

  /** Logs data to the SmartDashboard. */
  private void logData() {
    SmartDashboard.putString("Elevator State", state.toString());
    SmartDashboard.putNumber("Elevator Position", getEncoder().getPosition());
    SmartDashboard.putNumber("Elevator Speed", elevatorMotorSparkMax.get());
  }
}
