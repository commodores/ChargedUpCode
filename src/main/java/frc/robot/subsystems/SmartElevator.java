// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;

public class SmartElevator extends ProfiledPIDSubsystem {
  private final CANSparkMax elevatorMotor = new CANSparkMax(Constants.ElevatorConstants.elevatorMotorID, MotorType.kBrushless);
  public final TimeOfFlight elevatorSensor = new TimeOfFlight(Constants.ElevatorConstants.elevatorSensorID);

  private ElevatorFeedforward feedforward =
        new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG,
                           Constants.ElevatorConstants.kV, Constants.ElevatorConstants.kA);
  
  /** Creates a new SmartElevator. */

  public SmartElevator() {
    super(new ProfiledPIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD, new TrapezoidProfile.Constraints(
      Constants.ElevatorConstants.MAX_VEL, Constants.ElevatorConstants.MAX_ACC)));
      getController().setTolerance(1);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
      double kS = SmartDashboard.getNumber("Elevator FeedForward kS", 0);
      double kG = SmartDashboard.getNumber("Elevator FeedForward kG", 0);
      double kV = SmartDashboard.getNumber("Elevator FeedForward kV", 0);
      double kA = SmartDashboard.getNumber("Elevator FeedForward kA", 0);

      ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

      double ff = feedforward.calculate(setpoint.position, setpoint.velocity);

      elevatorMotor.setVoltage(MathUtil.clamp(output + ff, -12, 12));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return elevatorSensor.getRange();
  }

  public ElevatorFeedforward getFeedForward(){
    return feedforward;
  }

  public void setFeedForward(ElevatorFeedforward feedforward){
      this.feedforward = feedforward;
  }

  public void runElevator(double speed){
    elevatorMotor.set(speed);
  }

  public double getElevatorPosition(){
    return elevatorSensor.getRange();
  }

  
}
