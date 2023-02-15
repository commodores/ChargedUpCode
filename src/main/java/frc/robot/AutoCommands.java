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
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.High;
import frc.robot.commands.Nothing;
import frc.robot.subsystems.Swerve;

public class AutoCommands {

    private final Swerve swerve;
    public final Map<String, SequentialCommandGroup> autos;
    public final Map<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;

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


       // eventMap.put("BarrelMarker1", new PrintCommand("You are Barrel Racing!!!"));
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


        eventMap.put("shooterStrt", new PrintCommand("Start Shooter"));
        eventMap.put("intakeDown", new PrintCommand("Intake Down"));
        eventMap.put("intakeOn", new PrintCommand("Intake On"));
        eventMap.put("intakeOff", new PrintCommand("Intake Off"));
        eventMap.put("turnToTarget", new PrintCommand("Turn to Target"));
        eventMap.put("shoot", new PrintCommand("Shoot"));
        eventMap.put("event", new High(RobotContainer.m_Arm, RobotContainer.m_Elevator));
        //eventMap.put("shooterStart", new ShooterSetRPM(5000));
        //eventMap.put("intakeDown", new IntakeDown());
        //eventMap.put("intakeOn", new IntakeRun());
        //eventMap.put("intakeOff", new IntakeStop());
        //eventMap.put("turnToTarget", new DriveTurnToTarget());
        //eventMap.put("shoot", new ShooterShoot());
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