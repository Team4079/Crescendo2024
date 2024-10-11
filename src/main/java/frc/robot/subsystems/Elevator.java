package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.GlobalsValues.ElevatorGlobalValues;

/**
 * The {@link Elevator} class represents the elevator subsystem of the robot.
 * It controls the elevator and pass motors, and manages their states and positions.
 */
public class Elevator extends SubsystemBase {

  /** Enum representing the state of the elevator. */
  public enum ElevatorState {
    UP,
    DOWN
  }

  /** The SparkMax motor controller for the elevator motor. */
  private final CANSparkMax elevatorMotorSparkMax;
  /** The SparkMax motor controller for the pass motor. */
  private final CANSparkMax passMotorSparkMax;
  /** The current state of the elevator. */
  private ElevatorState state = ElevatorState.DOWN;

  /**
   * Constructs a new Elevator subsystem.
   * Initializes the motor controllers and their settings.
   */
  public Elevator() {
    elevatorMotorSparkMax =
        new CANSparkMax(ElevatorGlobalValues.ELEVATOR_NEO_ID, MotorType.kBrushless);
    passMotorSparkMax = new CANSparkMax(ElevatorGlobalValues.PASS_NEO_ID, MotorType.kBrushless);
    elevatorMotorSparkMax.restoreFactoryDefaults();
    elevatorMotorSparkMax.setIdleMode(IdleMode.kBrake);
    passMotorSparkMax.restoreFactoryDefaults();
    passMotorSparkMax.setIdleMode(IdleMode.kBrake);

    elevatorMotorSparkMax.setClosedLoopRampRate(ElevatorGlobalValues.closedLoopRampRate);
    passMotorSparkMax.setClosedLoopRampRate(ElevatorGlobalValues.passClosedLoopRampRate);
    getEncoder().setPosition(0);

    elevatorMotorSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    elevatorMotorSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    elevatorMotorSparkMax.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kForward, ElevatorGlobalValues.SOFTLIMIT_FOWARD);
    elevatorMotorSparkMax.setSoftLimit(
        CANSparkMax.SoftLimitDirection.kReverse, ElevatorGlobalValues.SOFTLIMIT_REVERSE);

    getElevatorMotorPIDController().setP(ElevatorGlobalValues.kP);
    getElevatorMotorPIDController().setI(ElevatorGlobalValues.kI);
    getElevatorMotorPIDController().setD(ElevatorGlobalValues.kD);

    getPassMotorPIDController().setP(ElevatorGlobalValues.PasskP);
    getPassMotorPIDController().setI(ElevatorGlobalValues.PasskI);
    getPassMotorPIDController().setD(ElevatorGlobalValues.PasskD);
    logData();
  }

  /**
   * This method is called periodically to update the elevator position based on its current state.
   * <p>If the state is UP, it sets the elevator position to the up position defined in ElevatorGlobalValues.</p>
   * If the state is DOWN, it sets the elevator position to the down position defined in ElevatorGlobalValues.
   */
  @Override
  public void periodic() {
    if (state == ElevatorState.UP) {
      setElevatorPosition(ElevatorGlobalValues.ELEVATOR_UP);
      setPassSpeed(0.5);
    } else if (state == ElevatorState.DOWN) {
      setElevatorPosition(ElevatorGlobalValues.ELEVATOR_DOWN);
      stopPassMotor();
    }
  }

  /**
   * Sets the state of the elevator.
   * @param state The state to set the elevator to.
   */
  public void setState(ElevatorState state) {
    this.state = state;
  }

  /**
   * Sets the speed of the pass motor.
   * @param speed The speed to set the pass motor to.
   */
  public void setPassSpeed(double speed) {
    passMotorSparkMax.set(speed);
  }

  /** Stops the pass motor. */
  public void stopPassMotor() {
    passMotorSparkMax.set(0);
  }

  /**
   * Sets the position of the elevator.
   *
   * @param position The position to set the elevator to.
   */
  public void setElevatorPosition(double position) {
    elevatorMotorSparkMax.getPIDController().setReference(position, CANSparkBase.ControlType.kPosition);
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
   * Gets the PID controller of the elevator's motor.
   *
   * @return The PID controller of the elevator.
   */
  public SparkPIDController getElevatorMotorPIDController() {
    return elevatorMotorSparkMax.getPIDController();
  }

  /**
   * Gets the PID controller of the pass motor.
   *
   * @return The PID controller of the pass motor.
   */
  public SparkPIDController getPassMotorPIDController() {
    return passMotorSparkMax.getPIDController();
  }

  /**
   * Gets the encoder of the elevator's motor.
   *
   * @return The encoder of the elevator.
   */
  public RelativeEncoder getEncoder() {
    return elevatorMotorSparkMax.getEncoder();
  }

  /** Logs data to the SmartDashboard. */
  private void logData() {
    SmartDashboard.putString("Elevator State", state.toString());
    SmartDashboard.putNumber("Elevator Position", getEncoder().getPosition());
  }
}
