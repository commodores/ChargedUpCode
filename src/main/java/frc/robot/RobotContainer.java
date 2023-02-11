package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
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
    private final Joystick driver = new Joystick(0);
    private final Joystick driverTwo = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons Controller 1 */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton intake  = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton release = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton lowerElevator = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton raiseElevator = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton bigTest = new JoystickButton(driver,XboxController.Button.kBack.value);


     /* Driver Buttons Controller 2 */
    private final JoystickButton raiseArm = new JoystickButton(driverTwo, XboxController.Button.kRightBumper.value);
    private final JoystickButton lowerArm = new JoystickButton(driverTwo, XboxController.Button.kLeftBumper.value);
    private final JoystickButton restpose = new JoystickButton(driverTwo, XboxController.Button.kB.value);
    private final JoystickButton armHigh = new JoystickButton(driverTwo, XboxController.Button.kY.value);
    private final JoystickButton armMid = new JoystickButton(driverTwo, XboxController.Button.kX.value);
    private final JoystickButton armLow = new JoystickButton(driverTwo, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake m_Intake = new Intake();
    private final Arm m_Arm = new Arm();
   // private final Elevator m_Elevator = new Elevator();
    public final Elevator m_Elevator = new Elevator();
   
    private final SendableChooser<SequentialCommandGroup> autoChooser;
    private final AutoCommands autos;
    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );
        autos = new AutoCommands(s_Swerve);
        autoChooser = new SendableChooser<>();
    
        Set<String> keys = autos.autos.keySet();
        autoChooser.setDefaultOption((String) keys.toArray()[1], autos.autos.get(keys.toArray()[1]));
        //keys.remove((String) keys.toArray()[0]);
    
        for (String i : autos.autos.keySet()) {
            autoChooser.addOption(i, autos.autos.get(i));
        }
    
        SmartDashboard.putData("Auto Selector", autoChooser);
    
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
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        intake.onTrue(new InstantCommand(() -> m_Intake.runIntake(-1)));
        intake.onFalse(new InstantCommand(() -> m_Intake.runIntake(0)));
        release.onTrue(new InstantCommand(() -> m_Intake.runIntake(.4)));
        release.onFalse(new InstantCommand(() -> m_Intake.runIntake(0)));

        lowerElevator.onTrue(new HighElevator(m_Elevator));

        lowerArm.onTrue(new HighArm(m_Arm));

        bigTest.onTrue(new High(m_Arm, m_Elevator));
    
    
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
