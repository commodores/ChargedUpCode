// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ManualArm extends CommandBase {
  private final Arm m_Arm;
  double speed;

  /** Creates a new ShelfArm. */
  public ManualArm(Arm subsystem, double speedManual) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Arm = subsystem;
    addRequirements(m_Arm);
    speed = speedManual;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Arm.manualArm(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Arm.manualArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
