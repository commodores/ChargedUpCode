// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Elevator extends SubsystemBase {
  
  private final CANSparkMax elevatorMotor;
  private final TimeOfFlight elevatorSensor;
  
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.elevatorMotorID, MotorType.kBrushless);
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setSmartCurrentLimit(20);
    elevatorMotor.setIdleMode(IdleMode.kBrake);

    elevatorSensor = new TimeOfFlight(Constants.ElevatorConstants.elevatorSensorID);



  }

  public void runElevator(double speed){
    elevatorMotor.set(speed);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Elevator Height: ", elevatorSensor.getRange());
  }
}
