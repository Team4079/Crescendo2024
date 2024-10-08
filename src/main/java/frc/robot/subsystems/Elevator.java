// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.utils.GlobalsValues;
import frc.robot.utils.GlobalsValues.ElevatorGlobalValues;


public class Elevator extends SubsystemBase {
  
    public enum ElevatorState{
      UP,
      DOWN
    }

    private final CANSparkMax elevatorMotorSparkMax;
    private final CANSparkMax passMotorSparkMax;
    private ElevatorState state = ElevatorState.DOWN;

    // Creates a new Elevator. 
    public Elevator() {
      elevatorMotorSparkMax = new CANSparkMax(ElevatorGlobalValues.ELEVATOR_NEO_ID, MotorType.kBrushless);
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

      elevatorMotorSparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, ElevatorGlobalValues.SOFTLIMIT_FOWARD);
      elevatorMotorSparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, ElevatorGlobalValues.SOFTLIMIT_REVERSE);

      getPIDController().setP(ElevatorGlobalValues.kP);
      getPIDController().setI(ElevatorGlobalValues.kI);
      getPIDController().setD(ElevatorGlobalValues.kD);
      
      getPIDController().setP(ElevatorGlobalValues.PasskP);
      getPIDController().setI(ElevatorGlobalValues.PasskI);
      getPIDController().setD(ElevatorGlobalValues.PasskD);
      logData();
    }
    
      // This method is called periodically to update the elevator position based on its current state.
      // If the state is UP, it sets the elevator position to the up position defined in ElevatorGlobalValues.
      // If the state is DOWN, it sets the elevator position to the down position defined in ElevatorGlobalValues.
     
    // @Override
    public void periodic() {
      if (state == ElevatorState.UP) {
        setElevatorPosition(ElevatorGlobalValues.ELEVATOR_UP);
        setPassSpeed(0.5);
      } else if (state == ElevatorState.DOWN) {
        setElevatorPosition(ElevatorGlobalValues.ELEVATOR_DOWN);
        stopPassMotor();
      }
    }

    /* 
     * Sets the state of the elevator.
     * @param state The state to set the elevator to.
     */

    public void setState(ElevatorState state) {
      this.state = state;
    }

    /**
     * Gets the current state of the elevator.
     * @return The current state of the elevator.
     */
     
    public ElevatorState getState() {
      return state;
    }

    public void setPassSpeed(double speed)
    {
      passMotorSparkMax.set(speed);
    }
    public void stopPassMotor()
    {
      passMotorSparkMax.set(0);
    }

    /**
     * Sets the position of the elevator.
     * @param position The position to set the elevator to.
     */
     
    public void setElevatorPosition(double position){

      elevatorMotorSparkMax.getPIDController().setReference(position, ControlType.kPosition);
    }
    /**
     * Gets the SparkMax motor of the elevator.
     * @return The SparkMax motor of the elevator.
     */

     public double getElevatorPosition(){
      return elevatorMotorSparkMax.getEncoder().getPosition();
     }
     
    public CANSparkMax getElevatorMotorSparkMax() {
      return elevatorMotorSparkMax;
    }
    /**
     * Gets the pid controller of the elevator's motor.
     * @return The pid controller of the elevator.
     */
     
    public SparkPIDController getPIDController() {
      return elevatorMotorSparkMax.getPIDController();
    }
    /**
     * Gets the encoder of the elevator's motor.
     * @return The encoder of the elevator.
     */
     
    public RelativeEncoder getEncoder() {
      return elevatorMotorSparkMax.getEncoder();
    }
    /**
     * Logs data to the SmartDashboard.
     */
    
     
    private void logData(){
      SmartDashboard.putString("Elevator State", state.toString());
      SmartDashboard.putNumber("Elevator Position", getEncoder().getPosition());
    }
  } 
