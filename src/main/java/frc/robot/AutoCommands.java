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
        List<PathPlannerTrajectory> Charge = PathPlanner.loadPathGroup("Charge", new PathConstraints(4, 3));
        autos.put("Charge Auto", new SequentialCommandGroup(
            getCommand(Charge)
        ));

         //Out of Com and Charge Auto/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         List<PathPlannerTrajectory> outAndCharge = PathPlanner.loadPathGroup("outAndCharge", new PathConstraints(4, 3));
         autos.put("Out And Charge", new SequentialCommandGroup(
             getCommand(outAndCharge)
         ));
        
         //Out of Com Center/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         List<PathPlannerTrajectory> outOfComCenter = PathPlanner.loadPathGroup("outOfComCenter", new PathConstraints(4, 3));
         autos.put("Out And Com Center", new SequentialCommandGroup(
             getCommand(outOfComCenter)
         ));

         //TwoPoint/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
         List<PathPlannerTrajectory> TwoPoint = PathPlanner.loadPathGroup("TwoPoint", new PathConstraints(4, 3));
         autos.put("TwoPoint", new SequentialCommandGroup(
             getCommand(TwoPoint)
         ));


        //Events////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        eventMap.put("runIntake", new AutoIntake(RobotContainer.m_Intake));
        eventMap.put("release", new AutoRelease(RobotContainer.m_Intake));
        eventMap.put("stop", new AutoStop(RobotContainer.m_Intake));
        eventMap.put("groundArm", new Ground(RobotContainer.m_Arm, RobotContainer.m_Elevator));
        eventMap.put("stowArm", new Stow(RobotContainer.m_Arm, RobotContainer.m_Elevator));
        eventMap.put("highShot", new High(RobotContainer.m_Arm, RobotContainer.m_Elevator));
        eventMap.put("midShot", new Mid(RobotContainer.m_Arm, RobotContainer.m_Elevator));
        eventMap.put("autoWait", new AutoWait());        
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
            swerve);

        return autoBuilder.fullAuto(pathGroup);
    }

    

}