// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.lib.util.RevPIDGains;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestArm extends SubsystemBase {
  private CANSparkMax m_motor;
  private RelativeEncoder m_encoder;
  private SparkMaxPIDController m_controller;
  private double m_setpoint;

  private TrapezoidProfile m_profile;
  private Timer m_timer;
  
  private TrapezoidProfile.State targetState;
  private double feedforward;
  private double manualValue;
  
  /** Creates a new ArmSubsystem. */
  public TestArm() {
    m_motor = new CANSparkMax(Constants.ArmConstants.testArmMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_motor.setInverted(false);
    m_motor.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);
    m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_motor.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.ArmConstants.kSoftLimitForward);
    m_motor.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.ArmConstants.kSoftLimitReverse);

    m_encoder = m_motor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, 42);
    m_encoder.setPositionConversionFactor(Constants.ArmConstants.kArmGearRatio);
    m_encoder.setVelocityConversionFactor(Constants.ArmConstants.kArmGearRatio);

    m_controller = m_motor.getPIDController();
    RevPIDGains.setSparkMaxGains(m_controller, Constants.ArmConstants.kArmPositionGains);

    m_motor.burnFlash();

    m_setpoint = Constants.ArmConstants.kHomePosition;

    m_timer = new Timer();
    m_timer.start();
    m_timer.reset();

    updateMotionProfile();
  }

  public void setTargetPosition(double _setpoint) {
    if (_setpoint != m_setpoint) {
      m_setpoint = _setpoint;
      updateMotionProfile();
    }
  }

  private void updateMotionProfile() {
    TrapezoidProfile.State state = new TrapezoidProfile.State(m_encoder.getPosition(), m_encoder.getVelocity());
    TrapezoidProfile.State goal = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.ArmConstants.kArmMotionConstraint, goal, state);
    m_timer.reset();
  }

  public void runAutomatic() {
    double elapsedTime = m_timer.get();
    if (m_profile.isFinished(elapsedTime)) {
      targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    }
    else {
      targetState = m_profile.calculate(elapsedTime);
    }

    feedforward = Constants.ArmConstants.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.ArmConstants.kArmZeroCosineOffset, targetState.velocity);
    m_controller.setReference(targetState.position, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }

  public void runManual(double _power) {
    //reset and zero out a bunch of automatic mode stuff so exiting manual mode happens cleanly and passively
    m_setpoint = m_encoder.getPosition();
    targetState = new TrapezoidProfile.State(m_setpoint, 0.0);
    m_profile = new TrapezoidProfile(Constants.ArmConstants.kArmMotionConstraint, targetState, targetState);
    //update the feedforward variable with the newly zero target velocity
    feedforward = Constants.ArmConstants.kArmFeedforward.calculate(m_encoder.getPosition()+Constants.ArmConstants.kArmZeroCosineOffset, targetState.velocity);
    m_motor.set(_power + (feedforward / 12.0));
    manualValue = _power;
  }

  @Override
  public void periodic() { // This method will be called once per scheduler run
    SmartDashboard.putNumber("Test Arm Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Test Arm Feed Forward", feedforward);
  }

  @Override
  public void simulationPeriodic() { // This method will be called once per scheduler run during simulation
    
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Final Setpoint",  () -> m_setpoint, null);
    builder.addDoubleProperty("Position", () -> m_encoder.getPosition(), null);
    builder.addDoubleProperty("Applied Output", () -> m_motor.getAppliedOutput(), null);
    builder.addDoubleProperty("Elapsed Time", () -> m_timer.get(), null);
    /*builder.addDoubleProperty("Target Position", () -> targetState.position, null);
    builder.addDoubleProperty("Target Velocity", () -> targetState.velocity, null);*/
    builder.addDoubleProperty("Feedforward", () -> feedforward, null);
    builder.addDoubleProperty("Manual Value", () -> manualValue, null);
    //builder.addDoubleProperty("Setpoint", () -> m_setpoint, (val) -> m_setpoint = val);
    //builder.addBooleanProperty("At Setpoint", () -> atSetpoint(), null);
    //addChild("Controller", m_controller);
  }
}