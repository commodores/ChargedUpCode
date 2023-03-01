package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.*;;

public class AutoCommands {

    private final Swerve swerve;
    public final Map<String, SequentialCommandGroup> autos;
    public final Map<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;
    private PathPlannerTrajectory trajectory;

    //Example Multi-Path
    

    public AutoCommands(Swerve swerve) {
        
        this.swerve = swerve;

        //Build Autos
        autos = new HashMap<String, SequentialCommandGroup>();
        eventMap = new HashMap<String, Command>();
        
        autos.put("nothing", new SequentialCommandGroup(
            new Nothing()
        ));
        
        /////Charge Auto//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        List<PathPlannerTrajectory> Charge = PathPlanner.loadPathGroup("Charge", new PathConstraints(1.25, 1.25));
        autos.put("Charge", new SequentialCommandGroup(
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            new Mid(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(3),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(1),
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            getCommand(Charge),
            new AutoBalanceCommand(RobotContainer.s_Swerve),
            new AutoLock(RobotContainer.s_Swerve)
        ));

         //RightSideOnePoint/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         List<PathPlannerTrajectory> RightSideOnePoint = PathPlanner.loadPathGroup("RightSideOnePoint", new PathConstraints(2, 1));
         autos.put("RightSideOnePoint", new SequentialCommandGroup(
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            new Mid(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(3),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(1),
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            getCommand(RightSideOnePoint)
         ));

         //TwoPoint/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         List<PathPlannerTrajectory> TwoPoint = PathPlanner.loadPathGroup("TwoPoint", new PathConstraints(3, 2));
         autos.put("TwoPoint", new SequentialCommandGroup(
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            new Mid(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(3),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(1),
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            getCommand(TwoPoint),
            new Mid(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(3.5),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(0.75),
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(2.0)
         ));

         //RightSideCharge/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         List<PathPlannerTrajectory> RightSideCharge = PathPlanner.loadPathGroup("RightSideCharge", new PathConstraints(3.5, 2));
         autos.put("RightSideCharge", new SequentialCommandGroup(
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            new Mid(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(3),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(1),
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            getCommand(RightSideCharge),
            new AutoBalanceCommand(RobotContainer.s_Swerve),
            new AutoLock(RobotContainer.s_Swerve)
            
         ));

         List<PathPlannerTrajectory> AutoBalanceTest = PathPlanner.loadPathGroup("AutoBalanceTest", new PathConstraints(1, 1));
         autos.put("AutoBalanceTest", new SequentialCommandGroup(
             getCommand(AutoBalanceTest),
             new AutoBalanceCommand(RobotContainer.s_Swerve),
             new AutoLock(RobotContainer.s_Swerve)
         ));

         List<PathPlannerTrajectory> OnePointCharge = PathPlanner.loadPathGroup("OnePointCharge", new PathConstraints(3, 2));
         autos.put("OnePointCharge", new SequentialCommandGroup(
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            new Mid(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(3),
            new AutoRelease(RobotContainer.m_Intake).withTimeout(1),
            new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(.5),
            getCommand(OnePointCharge),
            new AutoBalanceCommand(RobotContainer.s_Swerve),
            new AutoLock(RobotContainer.s_Swerve)
         ));
 

        //Events////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        eventMap.put("runIntake", new AutoIntake(RobotContainer.m_Intake).withTimeout(3));
        eventMap.put("release", new AutoRelease(RobotContainer.m_Intake));
        eventMap.put("stopIntake", new StopIntake(RobotContainer.m_Intake));
        eventMap.put("stop", new AutoStop(RobotContainer.m_Intake));
        eventMap.put("groundArm", new Ground(RobotContainer.m_Arm, RobotContainer.m_Elevator).withTimeout(2));
        eventMap.put("stowArm", new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator));
        eventMap.put("highShot", new High(RobotContainer.m_Arm, RobotContainer.m_Elevator));
        eventMap.put("midShot", new Mid(RobotContainer.m_Arm, RobotContainer.m_Elevator)); 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    }

    private Command getCommand(List<PathPlannerTrajectory>pathGroup) {                
        
        autoBuilder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetOdometry,
            Constants.Swerve.swerveKinematics,
            new PIDConstants(Constants.AutoConstants.kPXandYControllers, 0, 0),
            new PIDConstants(Constants.AutoConstants.kPThetaController, 0, 0),
            swerve::setModuleStates,
            eventMap,
            true,
            swerve);

        return autoBuilder.fullAuto(pathGroup);
    }

    

}