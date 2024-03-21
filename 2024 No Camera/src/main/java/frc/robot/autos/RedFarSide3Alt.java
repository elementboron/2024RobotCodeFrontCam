package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.AutoCommands.AutoWheels;
import frc.robot.commands.AutoCommands.StopRobotAutonomous;
import java.util.function.BooleanSupplier;
import frc.robot.commands.AutoCommands.TurnUntilTarget;
import frc.robot.commands.swerve.AutoSwerveAim;
import frc.robot.commands.swerve.AutoSwerveAimAtNote;
import frc.robot.commands.swerve.AutoSwerveAimOther;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.commands.HarvesterDriveStart;
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

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

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

public class RedFarSide3Alt extends SequentialCommandGroup {

    public RedFarSide3Alt(Swerve s_Swerve, PhotonVision s_Vision, LinearActuator mLinearActuator, ShooterWheels mWheels, IntakeDrive mIntake, IntakeWrist mWrist, CommonMethodExtensions methods){

        TrajectoryConfig config =
            new TrajectoryConfig(
                    5,
                    4)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory backUpInitial =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-0.25, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-0.5, 0, new Rotation2d(0)),
                config.setReversed(true));

        Trajectory secondBackUp =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-3, -0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-5.5, 3, new Rotation2d(-60 * (Math.PI/180))),
                config.setReversed(true));

        Trajectory backIntoSecond =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-0.25, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-1.5, 0, new Rotation2d(0)),
                config.setReversed(true));

        Trajectory driveToShootSecond =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(4, 1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(4.25, 2.5, new Rotation2d(40 * (Math.PI/180))),
                config.setReversed(false));

        Trajectory pickUpThird =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(0, 1.95)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-3.3, 2.1, new Rotation2d(0)),
                config.setReversed(true));

        Trajectory driveIntoThird =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-0.2, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-1, 0, new Rotation2d(0)),
                config.setReversed(true));

        Trajectory finalPickUp =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-0.25, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-0.75, 0, new Rotation2d(0)),
                config.setReversed(true));

        Trajectory fourthDriveTo =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-2.5, -0.1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-2.6, -1.75, new Rotation2d(0)),
                config.setReversed(true));


        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand BackUpInitial =
            new SwerveControllerCommand(
                backUpInitial,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand BackIntoSecond =
            new SwerveControllerCommand(
                backIntoSecond,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand ShootSecond =
            new SwerveControllerCommand(
                driveToShootSecond,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(1, 0, 0),
                new PIDController(1, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        SwerveControllerCommand SecondBackUp =
            new SwerveControllerCommand(
                secondBackUp,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand DriveToThird =
            new SwerveControllerCommand(
                pickUpThird,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand BackIntoThird =
            new SwerveControllerCommand(
                driveIntoThird,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand DriveToShoot =
            new SwerveControllerCommand(
                driveIntoThird,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        SwerveControllerCommand DriveToFourth =
            new SwerveControllerCommand(
                fourthDriveTo,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroHeading())),

            new ParallelCommandGroup(            
                new AutoWheels(mWheels, 0.3),            
                BackUpInitial,
                new ShooterToSetpoint(mLinearActuator, methods, -45).withTimeout(0.75)
            ),
            new StopRobotAutonomous(s_Swerve),
            new ParallelCommandGroup(
                new AutoSwerveAim(s_Swerve, s_Vision).withTimeout(0.2),
                new AutoShooterAimAtTarget(mLinearActuator, s_Vision, methods).withTimeout(0.2)
            ),
            new StopRobotAutonomous(s_Swerve),
            new AutoHarvesterDriveStart(mIntake, -0.6),
            new WaitCommand(0.1),
            new AutoHarvesterDriveStart(mIntake, 0),

            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroHeading())),
            new ParallelCommandGroup(
                new AutoHarvesterDriveStart(mIntake, 0.6),
                new ShooterToSetpoint(mLinearActuator, methods, -100),
                new SequentialCommandGroup(
                    new WaitCommand(1.5),
                    new WristToggle(mWrist, methods).withTimeout(0.1)
                ),
                SecondBackUp
            ),

            new StopRobotAutonomous(s_Swerve),

            new AutoSwerveAimAtNote(s_Swerve, s_Vision).withTimeout(0.5),

            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroHeading())),
            BackIntoSecond,
            new StopRobotAutonomous(s_Swerve),
            new WristToggle(mWrist, methods),

            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroHeading())),

            ShootSecond,
            new StopRobotAutonomous(s_Swerve),
            new ParallelCommandGroup(
                new AutoSwerveAim(s_Swerve, s_Vision).withTimeout(1),
                new AutoShooterAimAtTarget(mLinearActuator, s_Vision, methods).withTimeout(1)
            ),

            new StopRobotAutonomous(s_Swerve),

            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroHeading())),
            new AutoHarvesterDriveStart(mIntake, -0.6),
            new WaitCommand(0.3),
            new AutoHarvesterDriveStart(mIntake, 0),
            new ParallelCommandGroup(
                DriveToThird, 
                new SequentialCommandGroup(
                    new WaitCommand(2),
                    new WristToggle(mWrist, methods),
                    new AutoHarvesterDriveStart(mIntake, 0.6)  
                )
                              
            ),
            new StopRobotAutonomous(s_Swerve),

            new AutoSwerveAimAtNote(s_Swerve, s_Vision).withTimeout(0.5),

            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.zeroHeading())),
            BackIntoThird,
            new StopRobotAutonomous(s_Swerve)

            );
    }
}