package frc.robot;

import com.pathplanner.lib.auto.*;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Autonomous.AmpDrop;
import frc.robot.commands.Autonomous.AutoAdjustIntake;
import frc.robot.commands.Autonomous.SpinUp;
import frc.robot.commands.Blinkin.RunLEDs;
import frc.robot.commands.Climber.Arms.*;
import frc.robot.commands.Climber.Hooks.*;
import frc.robot.commands.Combo.*;
import frc.robot.commands.Combo.Manual.*;
import frc.robot.commands.Feeder.*;
import frc.robot.commands.Intake.*;
import frc.robot.commands.Shooter.*;
import frc.robot.commands.Swerve.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    private final PS5Controller m_driver = new PS5Controller(OIConstants.kDriverController);
    private final Joystick m_operator = new Joystick(OIConstants.kOperatorController);

    // Subsystems
    private final Swerve m_swerve = new Swerve();
    private final Arm m_arm = new Arm();
    private final Intake m_intake = new Intake();
    private final Feeder m_feeder = new Feeder();
    private final Limelight m_light = new Limelight();
    private final Shooter m_shooter = new Shooter(m_light);    
    private final Climber m_climber = new Climber();
    private final LED m_led = new LED();
    // private final HttpCamera m_intakeCamera = new HttpCamera("Intake", "http://10.60.78.107:5800");
    // private final HttpCamera m_shooterCamera = new HttpCamera("Shooter", "http://10.60.78.108:5800");


    //Shuffleboard & Auto
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Main");
    private final SendableChooser<Command> m_chooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_swerve.setDefaultCommand(
            new TeleopSwerve(
                () -> -m_driver.getLeftY(), 
                () -> -m_driver.getLeftX(), 
                () -> -m_driver.getRightX(), 
                () -> m_driver.getR1Button(),
                m_swerve
            )
        );

        m_led.setDefaultCommand(new RunLEDs(m_led, m_feeder));

        // Commands that will show in PathPlanner
        NamedCommands.registerCommand("runIntake", new AutoIntake(m_feeder, m_intake, m_arm, m_shooter, m_light));
        NamedCommands.registerCommand("stopIntake", new StopIntake(m_intake));
        NamedCommands.registerCommand("closeShot", new closeShot(m_arm, m_shooter, m_intake, m_feeder));
        NamedCommands.registerCommand("midShot", new middleShot(m_arm, m_shooter, m_intake, m_feeder));
        NamedCommands.registerCommand("reset", new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        NamedCommands.registerCommand("ampDrop", new AmpDrop(m_arm, m_intake, m_shooter, m_feeder));
        NamedCommands.registerCommand("autoShoot", new AutoShoot(m_shooter, m_swerve, m_light, m_feeder, m_arm, m_intake));
        NamedCommands.registerCommand("autoIntake", new AutoAdjustIntake(m_feeder, m_intake, m_arm, m_shooter, m_light, m_swerve));
        NamedCommands.registerCommand("spinUp", new SpinUp(m_arm, m_shooter, m_light));

        // Configure the button bindings
        configureButtonBindings();

        m_chooser = AutoBuilder.buildAutoChooser();

        m_tab.add("Auto Chooser", m_chooser);
        // m_tab.add(m_intakeCamera).withSize(3, 3).withPosition(8, 0);
        // m_tab.add(m_shooterCamera).withSize(2, 2).withPosition(1, 2);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Driver Buttons
        new JoystickButton(m_driver, PS5Controller.Button.kL1.value).whileTrue(new AutoIntake(m_feeder, m_intake, m_arm, m_shooter, m_light)).onFalse(new SetFeederSpeed(0, m_feeder));
        new JoystickButton(m_driver, PS5Controller.Button.kCircle.value).whileTrue(new AutoShoot(m_shooter, m_swerve, m_light, m_feeder, m_arm, m_intake)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        new JoystickButton(m_driver, PS5Controller.Button.kOptions.value).onTrue(new ZeroHeading(m_swerve));
        new JoystickButton(m_driver, PS5Controller.Button.kR1.value).whileTrue(new SlowDrive(m_swerve));
        new JoystickButton(m_driver, PS5Controller.Button.kTriangle.value).onTrue(new Yeet(m_arm, m_shooter, m_feeder, m_intake)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));

        //Operator Buttons
        // Arm Commands
        new JoystickButton(m_operator, 1).whileTrue(new ArmUp(m_arm));
        new JoystickButton(m_operator, 6).whileTrue(new ArmDown(m_arm));
        new JoystickButton(m_operator, 2).onTrue(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));

        // Intake Commands
        new JoystickButton(m_operator, 4).onTrue(new AutoIntake(m_feeder, m_intake, m_arm, m_shooter, m_light));
        new JoystickButton(m_operator, 5).onTrue(new SourceIntake(m_shooter, m_feeder, m_intake, m_light, m_arm));
        
        // Shooter Commands
        new JoystickButton(m_operator, 8).whileTrue(new ShortShot(m_arm, m_shooter, m_feeder, m_intake)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        new JoystickButton(m_operator, 9).whileTrue(new MidShot(m_arm, m_shooter, m_intake, m_feeder)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        new JoystickButton(m_operator, 10).whileTrue(new FarShot(m_arm, m_shooter, m_feeder, m_intake)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));

        // Climb Commands
        new JoystickButton(m_operator, 18).whileTrue(new RaiseArms(m_climber)).onFalse(new StopArms(m_climber));
        new JoystickButton(m_operator, 20).whileTrue(new LowerArms(m_climber));
        new JoystickButton(m_operator, 17).whileTrue(new RaiseHooks(m_climber)).onFalse(new StopHooks(m_climber));
        new JoystickButton(m_operator, 19).whileTrue(new LowerHooks(m_climber)).onFalse(new StopHooks(m_climber));
        new JoystickButton(m_operator, 14).whileTrue(new ForceArmsDown(m_climber)).onFalse(new StopArms(m_climber));
        new JoystickButton(m_operator, 16).whileTrue(new ForceHooksDown(m_climber)).onFalse(new StopHooks(m_climber));
        new JoystickButton(m_operator, 23).onTrue(new ResetHooks(m_climber));
        new JoystickButton(m_operator, 23).onTrue(new ResetArms(m_climber));

        // Amp Commands
        new JoystickButton(m_operator, 12).whileTrue(new SetArmAngle(ArmConstants.kAmpAngle, m_arm).alongWith(new IdleIntake(m_intake))).onFalse(new SetArmAngle(0, m_arm).alongWith(new StopIntake(m_intake)));
        new JoystickButton(m_operator, 24).onTrue(new SetFeederSpeed(20, m_feeder).alongWith(new SetShooterSpeed(30, m_shooter))).onFalse(new SetFeederSpeed(0, m_feeder).alongWith(new SetShooterSpeed(0, m_shooter)));

        // Misc. Commands
        // new JoystickButton(m_operator, 15).onTrue(new SetShooterSpeed(m_light.getTargetRPM(), m_shooter)).onFalse(new SetShooterSpeed(0, m_shooter));
        // new JoystickButton(m_operator, 7).onTrue(new SetFeederSpeed(10, m_feeder)).onFalse(new SetFeederSpeed(0, m_feeder));
        // new JoystickButton(m_operator, 11).whileTrue(new SetArmAngle(49.5, m_arm)).onFalse(new SetArmAngle(0, m_arm));
        // new JoystickButton(m_operator, 3).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
        // new JoystickButton(m_operator, 5).onTrue(new AutoAdjustIntake(m_feeder, m_intake, m_arm, m_shooter, m_light, m_swerve));
        // new JoystickButton(m_operator, 12).onTrue(new AmpDrop(m_arm, m_intake, m_shooter, m_feeder)).onFalse(new ReturnToBasic(m_arm, m_shooter, m_intake, m_feeder));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     1
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
}
