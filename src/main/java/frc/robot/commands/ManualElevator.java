// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;

public class ManualElevator extends CommandBase {
  private final Elevator m_Elevator;
  double speed;

  /** Creates a new ShelfArm. */
  public ManualElevator(Elevator subsystem, double speedManual) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_Elevator = subsystem;
    addRequirements(m_Elevator);
    speed = speedManual;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Elevator.manualElevator(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Elevator.manualElevator(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
