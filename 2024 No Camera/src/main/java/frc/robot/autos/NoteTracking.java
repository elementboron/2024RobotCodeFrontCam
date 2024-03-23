package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.AutoCommands.AutoWheels;
import frc.robot.commands.AutoCommands.StopRobotAutonomous;
import java.util.function.BooleanSupplier;
import frc.robot.commands.AutoCommands.TurnUntilTarget;
import frc.robot.commands.swerve.AutoSwerveAim;
import frc.robot.commands.swerve.AutoSwerveAimAtNote;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.commands.HarvesterDriveStart;
import frc.robot.commands.NoteCamActivate;
import frc.robot.commands.ShooterCamActivate;
import frc.robot.commands.ShooterStart;
import frc.robot.commands.ShooterToSetpoint;
import frc.robot.commands.WristToggle;
import frc.robot.commands.AutoCommands.AutoAimActuatorAtTarget;
import frc.robot.commands.AutoCommands.AutoHarvesterDriveStart;
import frc.robot.commands.AutoCommands.AutoShooterAimAtTarget;
import frc.robot.subsystems.IntakeDrive;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LinearActuator;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterWheels;
import frc.robot.subsystems.Swerve;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class NoteTracking extends SequentialCommandGroup {

    public NoteTracking(Swerve s_Swerve, PhotonVision s_Vision, LinearActuator mLinearActuator, ShooterWheels mWheels, IntakeDrive mIntake, IntakeWrist mWrist, CommonMethodExtensions methods){

        TrajectoryConfig config =
            new TrajectoryConfig(
                    5,
                    3)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory backUpInitial =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-0.1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-1, 0, new Rotation2d(0)),
                config.setReversed(true));



        
        var thetaController =
            new ProfiledPIDController(
                7, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand BackUpInitial =
            new SwerveControllerCommand(
                backUpInitial,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(3, 0, 0),
                new PIDController(3, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        SwerveControllerCommand PickUpSecond =
            new SwerveControllerCommand(
                backUpInitial,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(3, 0, 0),
                new PIDController(3, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroHeading())),
            new NoteCamActivate(s_Vision),
            new WaitCommand(1),
            BackUpInitial,
            new StopRobotAutonomous(s_Swerve),
            new AutoSwerveAimAtNote(s_Swerve, s_Vision).withTimeout(2),
            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroHeading())),
            PickUpSecond,
            new StopRobotAutonomous(s_Swerve),
            new ShooterCamActivate(s_Vision)

            
            
            

            );
    }
}