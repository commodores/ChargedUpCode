// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Ground extends ParallelCommandGroup {
  private final Elevator m_Elevator;
  private final Arm m_Arm;
  /** Creates a new Ground. */
  public Ground(Arm armSusbsystem, Elevator elevatorSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_Arm = armSusbsystem;
    m_Elevator = elevatorSubsystem;
    addCommands(new GroundArm(m_Arm), new GroundElevator(m_Elevator));
  }
}
