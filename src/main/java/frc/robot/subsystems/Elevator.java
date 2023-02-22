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
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Elevator extends SubsystemBase {

  private final CANSparkMax elevatorMotor;
  private SparkMaxLimitSwitch elevatorLimit;

  private SparkMaxPIDController elevatorPIDController;
  private RelativeEncoder elevatorEncoder;
  //private SparkMaxLimitSwitch elevatorLimit;
  
  double kP = Constants.ElevatorConstants.elevatorKP,
    kI = Constants.ElevatorConstants.elevatorKI,
    kD = Constants.ElevatorConstants.elevatorKD,
    kIz = Constants.ElevatorConstants.elevatorKIz,
    kFF = Constants.ElevatorConstants.elevatorKFF, 
    kMinOutput = Constants.ElevatorConstants.elevatorKMinOutput,
    kMaxOutput = Constants.ElevatorConstants.elevatorKMaxOutput,
    minVel = Constants.ElevatorConstants.elevatorMinVel,
    maxVel = Constants.ElevatorConstants.elevatorMaxVel,
    maxAcc = Constants.ElevatorConstants.elevatorMaxAcc,
    allowedErr = Constants.ElevatorConstants.elevatorAllowedErr;

  
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.elevatorMotorID, MotorType.kBrushless);

    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setSmartCurrentLimit(30);
    elevatorMotor.setIdleMode(IdleMode.kBrake);

    elevatorMotor.setSoftLimit(SoftLimitDirection.kForward, 126);
    elevatorMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

    elevatorMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // initialze PID controller and encoder objects
    elevatorPIDController = elevatorMotor.getPIDController();
    elevatorEncoder = elevatorMotor.getEncoder();
    elevatorLimit = elevatorMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    elevatorLimit.enableLimitSwitch(true);

    // set PID coefficients
    elevatorPIDController.setP(kP);
    elevatorPIDController.setI(kI);
    elevatorPIDController.setD(kD);
    elevatorPIDController.setIZone(kIz);
    elevatorPIDController.setFF(kFF);
    elevatorPIDController.setOutputRange(kMinOutput, kMaxOutput);

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
    elevatorPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    elevatorPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    elevatorPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    elevatorPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard
    //SmartDashboard.putNumber("Elevator P Gain", kP);
    //SmartDashboard.putNumber("Elevator I Gain", kI);
    //SmartDashboard.putNumber("Elevator D Gain", kD);
    //SmartDashboard.putNumber("Elevator I Zone", kIz);
    //SmartDashboard.putNumber("Elevator Feed Forward", kFF);
    //SmartDashboard.putNumber("Elevator Max Output", kMaxOutput);
    //SmartDashboard.putNumber("Elevator Min Output", kMinOutput);

    // display Smart Motion coefficients
    //SmartDashboard.putNumber("Elevator Max Velocity", maxVel);
    //SmartDashboard.putNumber("Elevator Min Velocity", minVel);
    //SmartDashboard.putNumber("Elevator Max Acceleration", maxAcc);
    //SmartDashboard.putNumber("Elevator Allowed Closed Loop Error", allowedErr);
    //SmartDashboard.putNumber("Elevator Position", 0);
  }

  public void setPosition(double targetPosition){
    elevatorPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);
  }

  public void manualElevator(double speed){
    elevatorMotor.set(speed);
  }

  public double getOutputCurrent() {
    return elevatorMotor.getOutputCurrent();
  }

  public double getPosition() {
    return elevatorEncoder.getPosition();
  }

  public void resetEncoder(){
    elevatorMotor.getEncoder().setPosition(0);
  }

  public boolean getLimitSwitch(){
    return elevatorLimit.isPressed();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Current", getOutputCurrent());
    SmartDashboard.putNumber("Elevator Position", getPosition());
    SmartDashboard.putBoolean("Elevator Limit", getLimitSwitch());

    //if(getLimitSwitch()){
    //  resetEncoder();
    //}
  }

    /*
    // This method will be called once per scheduler run
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("Elevator Set P Gain", 0);
    double i = SmartDashboard.getNumber("Elevator Set I Gain", 0);
    double d = SmartDashboard.getNumber("Elevator Set D Gain", 0);
    double iz = SmartDashboard.getNumber("Elevator Set I Zone", 0);
    double ff = SmartDashboard.getNumber("Elevator Set Feed Forward", 0);
    double max = SmartDashboard.getNumber("Elevator Set Max Output", 0);
    double min = SmartDashboard.getNumber("Elevator Set Min Output", 0);
    double maxV = SmartDashboard.getNumber("Elevator Set Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Elevator Set Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Elevator Set Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Elevator Set Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { elevatorPIDController.setP(p); kP = p; }
    if((i != kI)) { elevatorPIDController.setI(i); kI = i; }
    if((d != kD)) { elevatorPIDController.setD(d); kD = d; }
    if((iz != kIz)) { elevatorPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { elevatorPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      elevatorPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { elevatorPIDController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { elevatorPIDController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { elevatorPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { elevatorPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    
    double setPoint = SmartDashboard.getNumber("Set Position", 0);
    
      /**
       * As with other PID modes, Smart Motion is set by calling the
       * setReference method on an existing pid object and setting
       * the control type to kSmartMotion
       */
    /*
    elevatorPIDController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Output", elevatorMotor.getAppliedOutput());
    */
  
}
