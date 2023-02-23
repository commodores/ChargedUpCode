// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  private final Swerve m_Swerve;

  private double m_currentAngle;
  private double m_error;
  private double m_drivePower;
  private int m_counter;
  private boolean m_isLevel;

  public AutoBalance(Swerve subsystem) {
    m_Swerve = subsystem;
    m_currentAngle = 0;
    m_counter = 0;
    m_isLevel = false;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_currentAngle = m_Swerve.getElevationAngle();
    m_error = AutoConstants.desiredBalanceAngle - m_currentAngle;

    m_drivePower = Math.min(AutoConstants.balanceP * m_error, 1);
    // Limit max power
    if (Math.abs(m_drivePower) > 0.28) {
      m_drivePower = Math.copySign(0.28, m_drivePower);
    }

    if (Math.abs(m_error) < 0.5 && m_counter == 10) {
      m_isLevel = true;
    }
    if (Math.abs(m_error) < 0.5 && m_counter < 10) {
      m_counter++;
    }

    //m_Swerve.setModuleStates(m_drivePower, 0.0, 0.0);
    m_Swerve.drive(new Translation2d(m_drivePower,0), 0, true, true);
    
    SmartDashboard.putNumber("Gyro Angle", m_currentAngle);
    SmartDashboard.putNumber("Drive Power", m_drivePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_isLevel;
  }
}
