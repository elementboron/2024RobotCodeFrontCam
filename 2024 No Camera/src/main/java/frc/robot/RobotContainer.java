package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.*;
import frc.robot.commands.swerve.AutoSwerveAim;
import frc.robot.commands.swerve.AutoSwerveAimAtNote;
import frc.robot.commands.swerve.AutoSwerveAimOther;
//import frc.robot.commands.*;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.commands.AutoCommands.AutoHarvesterDriveStart;
import frc.robot.commands.AutoCommands.AutoShooterAimAtTarget;
import frc.robot.commands.AutoCommands.AutoWheels;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {



    /* Controllers */
    private final XboxController driveStick = new XboxController(0);
    private final XboxController operatorStick = new XboxController(1);

    /* Drive Controls */
    //Joysticks
    private final int leftXAxis = XboxController.Axis.kLeftX.value;
    private final int leftYAxis = XboxController.Axis.kLeftY.value;
    private final int rightXAxis = XboxController.Axis.kRightX.value;
    private final int rightStickPress = XboxController.Button.kRightStick.value;
    private final int leftStickPress = XboxController.Button.kLeftStick.value;
    private final int raiseClimbers = XboxController.Axis.kRightTrigger.value;
    private final int lowerClimbers = XboxController.Axis.kLeftTrigger.value;
    private final int rightBumper = XboxController.Button.kRightBumper.value;
    private final int leftBumper = XboxController.Button.kLeftBumper.value;
    private final int startButton = XboxController.Button.kStart.value;
    private final int backButton = XboxController.Button.kBack.value;
    private final int aButton = XboxController.Button.kA.value;
    private final int xButton = XboxController.Button.kX.value;
    private final int yButton = XboxController.Button.kY.value;
    private final int bButton = XboxController.Button.kB.value;


    //shooter buttons




    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driveStick, backButton);

    private final JoystickButton wristSetPoint = new JoystickButton(operatorStick, backButton);
    private final JoystickButton robotCentric = new JoystickButton(driveStick, aButton);
    private final JoystickButton WristToggle = new JoystickButton(driveStick, yButton);

    private final JoystickButton testShootButton = new JoystickButton(operatorStick, yButton);

    private final JoystickButton ShooterFire = new JoystickButton(driveStick, xButton);
    private final JoystickButton ShooterCharge = new JoystickButton(driveStick, rightBumper);
    private final JoystickButton AmpFire = new JoystickButton(operatorStick, rightBumper);

    private final JoystickButton ShooterIntake = new JoystickButton(driveStick, leftBumper);
    private final JoystickButton LaunchButton = new JoystickButton(operatorStick, leftBumper);
    private final JoystickButton ManualShoot = new JoystickButton(operatorStick, bButton);
    private final JoystickButton HarvesterFeed = new JoystickButton(driveStick, startButton);
    private final JoystickButton IntakeOn = new JoystickButton(driveStick, leftStickPress);

    public final JoystickButton NoteLock = new JoystickButton(driveStick, bButton);

    


    //Speed Controls
    private final double desiredspeed = 1;
    private final double desiredturnspeed = desiredspeed*0.75;

    private final double shooterWheelSpeed = 0.3;

    /* Subsystems */
    private final Swerve mSwerve = new Swerve();
    private final ShooterWheels m_Wheels = new ShooterWheels();
    private final IntakeDrive mIntakeDrive = new IntakeDrive();
    private final LinearActuator mLinearActuator = new LinearActuator();
    private final IntakeWrist mWrist = new IntakeWrist();
    private final CommonMethodExtensions methods = new CommonMethodExtensions();
    private final PhotonVision mVision = new PhotonVision();
    private final Climber mClimber = new Climber();
    private final Blinkin mBlinkin = new Blinkin();
    
    /* Limelight Values */


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        mBlinkin.setDefaultCommand(new AutoColor(mBlinkin));

        NamedCommands.registerCommand("Reset Odometry", new InstantCommand(()-> mSwerve.setPose(new Pose2d(1.2, 5.5, new Rotation2d(0)))));
        NamedCommands.registerCommand("Wrist Toggle", new WristToggle(mWrist, methods));
        NamedCommands.registerCommand("Intake In", new AutoHarvesterDriveStart(mIntakeDrive, 0.6));
        NamedCommands.registerCommand("Intake Out", new AutoHarvesterDriveStart(mIntakeDrive, -0.6));
        NamedCommands.registerCommand("Aim Swerve At Speaker", new AutoSwerveAim(mSwerve, mVision));
        NamedCommands.registerCommand("Other Aim Swerve At Speaker", new AutoSwerveAimOther(mSwerve, mVision));

        NamedCommands.registerCommand("Aim Shooter At Speaker", new AutoShooterAimAtTarget(mLinearActuator, mVision, methods));
        NamedCommands.registerCommand("Shooter To Setpoint", new ShooterToSetpoint(mLinearActuator, methods, 60));
        NamedCommands.registerCommand("Activate Shooter", new AutoWheels(m_Wheels, 0.3));
        NamedCommands.registerCommand("Activate Slow Shooter", new AutoWheels(m_Wheels, 0.25));
        NamedCommands.registerCommand("Shooter To -45", new RepeatCommand(new ShooterToSetpoint(mLinearActuator, methods, -48)));
        NamedCommands.registerCommand("Shooter To -100", new RepeatCommand(new ShooterToSetpoint(mLinearActuator, methods, -75)));


       







        mSwerve.setDefaultCommand(
            new TeleopSwerve(
                mSwerve, 
                mVision,  
                () -> Math.pow(-desiredspeed*driveStick.getRawAxis(leftYAxis), 3), 
                () -> Math.pow(-desiredspeed*driveStick.getRawAxis(leftXAxis), 3),
                () -> -desiredturnspeed*driveStick.getRawAxis(rightXAxis),
                () -> robotCentric.getAsBoolean(),
                () -> ShooterCharge.getAsBoolean(),
                () -> NoteLock.getAsBoolean(),
                mWrist
            )
        );

        m_Wheels.setDefaultCommand(new ShooterStop(m_Wheels, 0 , 0));
        mIntakeDrive.setDefaultCommand(new HarvesterDriveStart(mIntakeDrive, 0));
        mWrist.setDefaultCommand(new WristDrive(mWrist, 0));



        mLinearActuator.setDefaultCommand(
            new StartLinearActuator(
                mLinearActuator,
                mVision,
                () -> operatorStick.getRawAxis(leftYAxis),
                () -> operatorStick.getRawAxis(leftXAxis)
            )
        );


        mClimber.setDefaultCommand(
            new ActivateClimbMotors(
                mClimber,
                () -> operatorStick.getRawAxis(raiseClimbers),
                () -> operatorStick.getRawAxis(lowerClimbers)
            )
        );

        mWrist.setDefaultCommand(
            new WristStop(mWrist)
            );
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
        zeroGyro.onTrue(new InstantCommand(() -> mSwerve.zeroHeading()));

        //shooter commands
        ShooterCharge.whileTrue(new ShooterStart(m_Wheels, mVision));
        ShooterCharge.onTrue(new ShooterCamActivate(mVision));
        ShooterIntake.whileTrue(new ShooterTuning(m_Wheels, -0.1, -0.1));
        ShooterIntake.whileTrue(new HarvesterDriveStart(mIntakeDrive, 0.5));

        ShooterCharge.whileTrue(new RepeatCommand(new ShooterAimAtTarget(mLinearActuator, mVision)));
        //ShooterCharge.whileTrue(new RepeatCommand(new ShooterToSetpoint(mLinearActuator, methods, 0)));

        LaunchButton.whileTrue(new ShooterTuning(m_Wheels, 0.25, 0.21).alongWith(new TeleopShooterToSetpoint(mLinearActuator, methods, -30)));
        ManualShoot.whileTrue(new ShooterTuning(m_Wheels, 0.25, 0.21).alongWith(new TeleopShooterToSetpoint(mLinearActuator, methods, 38)));

        //intake commands
        //IntakeOn.whileTrue(new HarvesterDriveStart(m_HarvesterDrive, .6));
        //NoteLock.whileTrue(new HarvesterDriveStart(m_HarvesterDrive, 0.6));
        ShooterFire.whileTrue(new HarvesterDriveStart(mIntakeDrive, -0.6));
        WristToggle.onTrue(new WristToggle(mWrist, methods));
        //NoteLock.onTrue(new NoteCamActivate(mVision));

        //AmpFire.whileTrue(new ShooterAmp(m_Wheels, 0.03, 0.11, 0.2).alongWith(new TeleopShooterToSetpoint(mLinearActuator, methods, 38.7)));
        AmpFire.whileTrue(new ShooterAmp(m_Wheels, 0.03, 0.11, 0.2).alongWith(new TeleopShooterToSetpoint(mLinearActuator, methods, 36.7)));

       //AmpFire.whileTrue(new ShooterAmp(m_Wheels, 0.06, 0.17, 0.2));

        //testShootButton.whileTrue(new NoteCamActivate(mVision));
        //testShootButton.whileTrue(new AutoSwerveAimAtNote(s_Swerve, mVision));
        //NoteLock.onTrue(new NoteCamActivate(mVision));
        //NoteLock.onFalse(new ShooterCamActivate(mVision));

           
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

            // Load the path you want to follow using its name in the GUI
        //PathPlannerPath path = PathPlannerPath.fromPathFile("New Path");
        //return AutoBuilder.followPath(path);
        return AutoBuilder.buildAuto("Time Save Red");
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        //return AutoBuilder.followPath(path);
        //return new PathPlannerAuto("DriveForward");
        //return new Middle2Piece( s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);
        //return new BlueAmpSideNew(s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);
        //return new NoteTracking(s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);
        //return null;
        //return new RotateLeft2(s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);
        //return new RedAmpSideTesting(s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);
        //return new FivePieceAuto(s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);
        //return new RedFarSide3Alt(s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);

    }
}
