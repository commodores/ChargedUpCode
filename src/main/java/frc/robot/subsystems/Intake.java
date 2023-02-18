// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;




public class Intake extends SubsystemBase {
  

  private final CANSparkMax intakeMotor;
  

  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new CANSparkMax(Constants.IntakeConstants.intakeMotorID, MotorType.kBrushless);
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.setIdleMode(IdleMode.kBrake);

    runIntakeSpeed(-.01);
    
  }

  public void runIntakeSpeed(double speed){
      intakeMotor.set(speed);
  }

  public void runIntakeVolts(double volts){
    intakeMotor.setVoltage(volts);
}

  public double getOutputCurrent() {
    return intakeMotor.getOutputCurrent();
  }

 


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Current", getOutputCurrent());
   
  }

}
