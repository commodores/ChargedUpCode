package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    private final XboxController driverTwo = new XboxController(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons Controller 1 */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);    
    private final JoystickButton raiseArm = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton lowerArm = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton raiseElevator = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton lowerElevator = new JoystickButton(driver, XboxController.Button.kBack.value); 
    

    /* Driver Buttons Controller 2 */
    private final JoystickButton intake  = new JoystickButton(driverTwo, XboxController.Button.kRightBumper.value);
    private final JoystickButton release = new JoystickButton(driverTwo, XboxController.Button.kLeftBumper.value);  
    private final JoystickButton stow = new JoystickButton(driverTwo, XboxController.Button.kB .value);
    private final JoystickButton ground = new JoystickButton(driverTwo, XboxController.Button.kA.value);
    private final JoystickButton mid = new JoystickButton(driverTwo, XboxController.Button.kX.value);
    private final JoystickButton high = new JoystickButton(driverTwo, XboxController.Button.kY.value);
    private final JoystickButton shelf = new JoystickButton(driverTwo, XboxController.Button.kStart.value);
    private final JoystickButton resetArm = new JoystickButton(driverTwo, XboxController.Button.kBack.value);


    /* Subsystems */
    public final static Swerve s_Swerve = new Swerve();
    public final static Intake m_Intake = new Intake();
    public final static Arm m_Arm = new Arm();
    public final static Elevator m_Elevator = new Elevator();
   
    private final SendableChooser<SequentialCommandGroup> autoChooser;
    private final AutoCommands autos;    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis)*.75, 
                () -> robotCentric.getAsBoolean()
            )
        );

       // m_Intake.setDefaultCommand(new IntakeHold(m_Intake));

        autos = new AutoCommands(s_Swerve);
        autoChooser = new SendableChooser<>();
    
        Set<String> keys = autos.autos.keySet();
        autoChooser.setDefaultOption((String) keys.toArray()[1], autos.autos.get(keys.toArray()[1]));
        //keys.remove((String) keys.toArray()[0]);
    
        for (String i : autos.autos.keySet()) {
            autoChooser.addOption(i, autos.autos.get(i));
        }
    
        SmartDashboard.putData("Auto Selector", autoChooser);

        SmartDashboard.putData("Reset Elevator", new ResetElevator(m_Elevator).withTimeout(.1));

        SmartDashboard.putData("Balance", new AutoBalance(s_Swerve));
    
        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /* Driver 1 Buttons */

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));//----Y Button
        
        raiseArm.onTrue(new ManualArm(m_Arm, .3));//----Right Bumper
        raiseArm.onFalse(new ManualArm(m_Arm, 0).withTimeout(0.1));
        lowerArm.onTrue(new ManualArm(m_Arm, -.3));//----Left Bumper
        lowerArm.onFalse(new ManualArm(m_Arm, 0).withTimeout(0.1));

        raiseElevator.onTrue(new ManualElevator(m_Elevator, .3));//----Start Button
        raiseElevator.onFalse(new ManualElevator(m_Elevator, 0).withTimeout(0.1));
        lowerElevator.onTrue(new ManualElevator(m_Elevator, -.3));//----Back Button
        lowerElevator.onFalse(new ManualElevator(m_Elevator, 0).withTimeout(0.1));
        
        /* Driver 2 Buttons */        

        intake.onTrue(new InstantCommand(() -> m_Intake.runIntakeSpeed(-1)));//----Right Bumper
        intake.onFalse(new InstantCommand(() -> m_Intake.runIntakeSpeed(-.02)));
        release.onTrue(new InstantCommand(() -> m_Intake.runIntakeSpeed(.35)));//----Left Bumper
        release.onFalse(new InstantCommand(() -> m_Intake.runIntakeSpeed(-.02)));
        
        stow.onTrue(new Stow(m_Arm, m_Elevator));//----B Button
        ground.onTrue(new Ground(m_Arm, m_Elevator));//----A Button
        mid.onTrue(new Mid(m_Arm, m_Elevator));//----X Button
        high.onTrue(new High(m_Arm, m_Elevator));//----Y Button
        shelf.onTrue(new Shelf(m_Arm, m_Elevator));//----Start Button
        resetArm.onTrue(new ResetArm(m_Arm));//-----Back Button

        //raiseArm.onTrue(new InstantCommand(() -> m_TestArm.setGoal(0)));
        //lowerArm.onTrue(new InstantCommand(() -> m_TestArm.setGoal(-10)));
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
       // return new exampleAuto(s_Swerve);
       return autoChooser.getSelected();
    }
}
