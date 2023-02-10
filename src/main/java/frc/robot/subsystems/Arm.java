// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm extends SubsystemBase {

  private final CANSparkMax armMotor;

  private SparkMaxPIDController armPIDController;
  private RelativeEncoder armEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  
  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    armMotor.setSmartCurrentLimit(20);
    armMotor.setIdleMode(IdleMode.kBrake);

    // initialze PID controller and encoder objects
    armPIDController = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();

    // PID coefficients
    kP = 0.00005; 
    kI = 0.000001;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000156; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // Smart Motion Coefficients
    maxVel = 2000; // rpm
    maxAcc = 1500;

    // set PID coefficients
    armPIDController.setP(kP);
    armPIDController.setI(kI);
    armPIDController.setD(kD);
    armPIDController.setIZone(kIz);
    armPIDController.setFF(kFF);
    armPIDController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * Smart Motion coefficients are set on a SparkMaxPIDController object
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * the pid controller in Smart Motion mode
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * RPM of the pid controller in Smart Motion mode
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * of the pid controller in Smart Motion mode
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * error for the pid controller in Smart Motion mode
     */
    int smartMotionSlot = 0;
    armPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    armPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    armPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    armPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Arm P Gain", kP);
    SmartDashboard.putNumber("Arm I Gain", kI);
    SmartDashboard.putNumber("Arm D Gain", kD);
    SmartDashboard.putNumber("Arm I Zone", kIz);
    SmartDashboard.putNumber("Arm Feed Forward", kFF);
    SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
    SmartDashboard.putNumber("Arm Min Output", kMinOutput);

    // display Smart Motion coefficients
    SmartDashboard.putNumber("Arm Max Velocity", maxVel);
    SmartDashboard.putNumber("Arm Min Velocity", minVel);
    SmartDashboard.putNumber("Arm Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Arm Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Arm Set Position", 0);
    SmartDashboard.putNumber("Arm Set Velocity", 0);
  }

  public void armToPosition(double setPoint){
    armPIDController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Arm Set P Gain", 0);
    double i = SmartDashboard.getNumber("Arm Set I Gain", 0);
    double d = SmartDashboard.getNumber("Arm Set D Gain", 0);
    double iz = SmartDashboard.getNumber("Arm Set I Zone", 0);
    double ff = SmartDashboard.getNumber("Arm Set Feed Forward", 0);
    double max = SmartDashboard.getNumber("Arm Set Max Output", 0);
    double min = SmartDashboard.getNumber("Arm Set Min Output", 0);
    double maxV = SmartDashboard.getNumber("Arm Set Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Arm Set Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Arm Set Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Arm Set Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { armPIDController.setP(p); kP = p; }
    if((i != kI)) { armPIDController.setI(i); kI = i; }
    if((d != kD)) { armPIDController.setD(d); kD = d; }
    if((iz != kIz)) { armPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { armPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      armPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { armPIDController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { armPIDController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { armPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { armPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    /*
    double setPoint, processVariable;

    setPoint = SmartDashboard.getNumber("Set Position", 0);
    
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
    /*
    armPIDController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
    processVariable = armEncoder.getPosition();

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Process Variable", processVariable);
    SmartDashboard.putNumber("Output", armMotor.getAppliedOutput());
    */
  }
}
