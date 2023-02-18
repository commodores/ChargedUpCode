// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm extends SubsystemBase {

  private final CANSparkMax armMotor;

  private SparkMaxPIDController armPIDController;
  private RelativeEncoder armEncoder;
  private SparkMaxLimitSwitch armLimit;
  
  double kP = Constants.ArmConstants.armKP,
    kI = Constants.ArmConstants.armKI,
    kD = Constants.ArmConstants.armKD,
    kIz = Constants.ArmConstants.armKIz,
    kFF = Constants.ArmConstants.armKFF, 
    kMinOutput = Constants.ArmConstants.armKMinOutput,
    kMaxOutput = Constants.ArmConstants.armKMaxOutput,
    minVel = Constants.ArmConstants.armMinVel,
    maxVel = Constants.ArmConstants.armMaxVel,
    maxAcc = Constants.ArmConstants.armMaxAcc,
    allowedErr = Constants.ArmConstants.armAllowedErr;

  
  /** Creates a new Arm. */
  public Arm() {
    armMotor = new CANSparkMax(Constants.ArmConstants.armMotorID, MotorType.kBrushless);

    armMotor.restoreFactoryDefaults();
    armMotor.setSmartCurrentLimit(30);
    armMotor.setIdleMode(IdleMode.kBrake);

    //armMotor.setSoftLimit(SoftLimitDirection.kForward, 5);
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, -61);

    //armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // initialze PID controller and encoder objects
    armPIDController = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();
    armLimit = armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    armLimit.enableLimitSwitch(true);

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
    //SmartDashboard.putNumber("Arm P Gain", kP);
    //SmartDashboard.putNumber("Arm I Gain", kI);
    //SmartDashboard.putNumber("Arm D Gain", kD);
    //SmartDashboard.putNumber("Arm I Zone", kIz);
    //SmartDashboard.putNumber("Arm Feed Forward", kFF);
    //SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
    //SmartDashboard.putNumber("Arm Min Output", kMinOutput);

    // display Smart Motion coefficients
    //SmartDashboard.putNumber("Arm Max Velocity", maxVel);
    //SmartDashboard.putNumber("Arm Min Velocity", minVel);
    //SmartDashboard.putNumber("Arm Max Acceleration", maxAcc);
    //SmartDashboard.putNumber("Arm Allowed Closed Loop Error", allowedErr);
    //SmartDashboard.putNumber("Arm Position", 0);
  }

  public void setPosition(double targetPosition){
    armPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);    
  }

  public void manualArm(double speed){
    armMotor.set(speed);
  }

  public double getOutputCurrent(){
    return armMotor.getOutputCurrent();
  }

  public double getPosition(){
    return armEncoder.getPosition();
  }

  public void resetEncoder(){
    armMotor.getEncoder().setPosition(0);
  }

  public boolean getLimitSwitch(){
    return armLimit.isPressed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Current", getOutputCurrent());
    SmartDashboard.putNumber("Arm Position", getPosition());
    SmartDashboard.putBoolean("Arm Limit", getLimitSwitch());

    if(getLimitSwitch()){
        resetEncoder();;
    }
  }
}
