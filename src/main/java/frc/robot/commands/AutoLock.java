package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoLock extends CommandBase {    
    private Swerve m_Swerve;    
    public AutoLock(Swerve subsystem) {
        m_Swerve = subsystem;
        addRequirements(m_Swerve);        
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0.05;
        m_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), rotationVal * Constants.Swerve.maxAngularVelocity, true, true);
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}