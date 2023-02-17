// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class TestArm extends TrapezoidProfileSubsystem {
  
  private final CANSparkMax armMotor;
  private final ArmFeedforward m_feedForward = new ArmFeedforward(Constants.ArmConstants.armKs,Constants.ArmConstants.armKg,Constants.ArmConstants.armKv,Constants.ArmConstants.armKa);
  private SparkMaxPIDController armPIDController;
  private RelativeEncoder armEncoder;
  private SparkMaxLimitSwitch armLimit;
  
  /** Creates a new TestArm. */
  public TestArm() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(Constants.ArmConstants.armMaxVel, Constants.ArmConstants.armMaxAcc),
        // The initial position of the mechanism
        0);

        armMotor = new CANSparkMax(Constants.ArmConstants.armMotorIDTest, MotorType.kBrushless);
        armMotor.restoreFactoryDefaults();
        armMotor.setSmartCurrentLimit(30);
        armMotor.setIdleMode(IdleMode.kBrake);

        armPIDController = armMotor.getPIDController();
        armPIDController.setP(Constants.ArmConstants.armKP);

        armEncoder = armMotor.getEncoder();
        armEncoder.setPosition(0);

        armLimit = armMotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Arm Current", getOutputCurrent());
    SmartDashboard.putNumber("Arm Position", getPosition());

    if(getLimitSwitch()){
        armEncoder.setPosition(0);
    }

  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedForward.calculate(state.position, state.velocity);
    //armPIDController.setFF(feedforward);
    armPIDController.setReference(state.position, CANSparkMax.ControlType.kPosition,0, feedforward / 12.0);
  }

  public double getOutputCurrent(){
    return armMotor.getOutputCurrent();
  }

  public double getPosition(){
    return armEncoder.getPosition();
  }

  public boolean getLimitSwitch(){
    return armLimit.isPressed();
  }
}
