package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.*;
//import frc.robot.commands.*;
import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

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

    //swerve buttons
    private final int zerogyro = XboxController.Button.kBack.value;
    private final int robotcentric = XboxController.Button.kA.value;
    private final int shooterFire = XboxController.Button.kX.value;
    private final int wristToggle = XboxController.Button.kY.value;
    private final int noteLock = XboxController.Button.kRightStick.value;
    private final int intakeOn = XboxController.Button.kLeftStick.value;

    //shooter buttons
    private final int activateShooterWheels = XboxController.Button.kRightBumper.value;
    private final int reverseShooterWheels = XboxController.Button.kLeftBumper.value;

    //harvester buttons
    private final int startHarvesterDrive = XboxController.Button.kB.value;
    private final int reverseHarvesterDrive = XboxController.Button.kStart.value;

    //linear actuator buttons
    private final int raiseLinearActuator = XboxController.Axis.kRightTrigger.value;
    private final int lowerLinearActuator = XboxController.Axis.kLeftTrigger.value;

    //wrist buttons
    private final int raiseWrist = XboxController.Axis.kRightTrigger.value;
    private final int lowerWrist = XboxController.Axis.kLeftTrigger.value;

    //climber buttons
    private final int raiseClimbers = XboxController.Axis.kRightTrigger.value;
    private final int lowerClimbers = XboxController.Axis.kLeftTrigger.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driveStick, zerogyro);

    private final JoystickButton wristSetPoint = new JoystickButton(operatorStick, zerogyro);
    private final JoystickButton robotCentric = new JoystickButton(driveStick, robotcentric);
    private final JoystickButton WristToggle = new JoystickButton(driveStick, wristToggle);

    private final JoystickButton ShooterFire = new JoystickButton(driveStick, shooterFire);
    private final JoystickButton ShooterCharge = new JoystickButton(driveStick, activateShooterWheels);
    private final JoystickButton AmpFire = new JoystickButton(operatorStick, activateShooterWheels);

    private final JoystickButton ShooterIntake = new JoystickButton(driveStick, reverseShooterWheels);
    private final JoystickButton HarvesterIntake = new JoystickButton(driveStick, startHarvesterDrive);
    private final JoystickButton HarvesterFeed = new JoystickButton(driveStick, reverseHarvesterDrive);
    private final JoystickButton IntakeOn = new JoystickButton(driveStick, intakeOn);

    public final JoystickButton NoteLock = new JoystickButton(driveStick, noteLock);

    


    //Speed Controls
    private final double desiredspeed = 1;
    private final double desiredturnspeed = desiredspeed*0.75;

    private final double shooterWheelSpeed = 0.3;

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ShooterWheels m_Wheels = new ShooterWheels();
    private final IntakeDrive m_HarvesterDrive = new IntakeDrive();
    private final LinearActuator mLinearActuator = new LinearActuator();
    private final IntakeWrist mWrist = new IntakeWrist();
    private final CommonMethodExtensions methods = new CommonMethodExtensions();
    private final PhotonVision mVision = new PhotonVision();
    private final Climber mClimber = new Climber();
    
    /* Limelight Values */


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
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
        m_HarvesterDrive.setDefaultCommand(new HarvesterDriveStart(m_HarvesterDrive, 0));
        mWrist.setDefaultCommand(new WristDrive(mWrist, 0));


        mLinearActuator.setDefaultCommand(
            new StartLinearActuator(
                mLinearActuator,
                mVision,
                () -> driveStick.getRawAxis(raiseLinearActuator),
                () -> driveStick.getRawAxis(lowerLinearActuator)
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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        //shooter commands
        ShooterCharge.whileTrue(new ShooterStart(m_Wheels, mVision));
        ShooterIntake.whileTrue(new ShooterTuning(m_Wheels, -0.1, -0.1));
        ShooterIntake.whileTrue(new HarvesterDriveStart(m_HarvesterDrive, 0.5));

        ShooterCharge.whileTrue(new RepeatCommand(new ShooterAimAtTarget(mLinearActuator, mVision)));
        //ShooterCharge.whileTrue(new RepeatCommand(new ShooterToSetpoint(mLinearActuator, methods, 0)));

        //intake commands
        IntakeOn.whileTrue(new HarvesterDriveStart(m_HarvesterDrive, .6));
        NoteLock.whileTrue(new HarvesterDriveStart(m_HarvesterDrive, 0.6));
        ShooterFire.whileTrue(new HarvesterDriveStart(m_HarvesterDrive, -.6));
        WristToggle.onTrue(new WristToggle(mWrist, methods));

        AmpFire.whileTrue(new ShooterAmp(m_Wheels, 0.05, 0.09, 0.2));

           
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        //PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        //return AutoBuilder.followPath(path);
        //return new Middle2Piece( s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);
        return new Left2Piece(s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);
        //return new RotateLeft2(s_Swerve, mVision, mLinearActuator, m_Wheels, m_HarvesterDrive, mWrist, methods);

    }
}
