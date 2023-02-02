// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Elevator extends SubsystemBase {
  
  private final CANSparkMax elevator;
  
  /** Creates a new Elevator. */
  public Elevator() {
    elevator = new CANSparkMax(1, MotorType.kBrushless);
    elevator.setIdleMode(IdleMode.kBrake);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
