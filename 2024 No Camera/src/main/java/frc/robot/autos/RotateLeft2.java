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

public class RotateLeft2 extends SequentialCommandGroup {

    public RotateLeft2(Swerve s_Swerve, PhotonVision s_Vision, LinearActuator mLinearActuator, ShooterWheels mWheels, IntakeDrive mIntake, IntakeWrist mWrist, CommonMethodExtensions methods){

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
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

        Trajectory pickUpSecondTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(-0.1, 0)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(-0.5, 0, new Rotation2d(0)),
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

        SwerveControllerCommand PickUpSecond =
            new SwerveControllerCommand(
                backUpInitial,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);

        addCommands(
            new InstantCommand((() -> s_Swerve.setPose(backUpInitial.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.OneEightyGyro(0))),
            new AutoWheels(mWheels, 0.3)
            /*
            BackUpInitial.until(s_Vision.hasTargetSupplier()),

            new AutoSwerveAim(s_Swerve, s_Vision).until(() -> isFinished()),
            new StopRobotAutonomous(s_Swerve),
            new AutoShooterAimAtTarget(mLinearActuator, s_Vision, methods),
            new WaitCommand(0.5),
            new AutoHarvesterDriveStart(mIntake, -0.6),
            new WaitCommand(0.5),

            //new AutoSwerveAimAtNote(s_Swerve, s_Vision, 1).until(() -> isFinished()),
            new InstantCommand((() -> s_Swerve.setPose(pickUpSecondTrajectory.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.OneEightyGyro(0))),
            new ParallelCommandGroup(
                new SequentialCommandGroup(
                   new WaitCommand(0.1),
                    PickUpSecond 
                ),
                new WristToggle(mWrist, methods),
                new AutoHarvesterDriveStart(mIntake, 0.6)
            ),
            new WristToggle(mWrist, methods),
            new AutoHarvesterDriveStart(mIntake, 0),
            new StopRobotAutonomous(s_Swerve),

            new AutoSwerveAim(s_Swerve, s_Vision).until(() -> isFinished()),

            new InstantCommand((() -> s_Swerve.setPose(pickUpSecondTrajectory.getInitialPose()))),
            new InstantCommand((() -> s_Swerve.OneEightyGyro(0))),

            new AutoShooterAimAtTarget(mLinearActuator, s_Vision, methods),
            new WaitCommand(0.5),
            new AutoHarvesterDriveStart(mIntake, -0.6),
            new WaitCommand(0.5),
            new AutoHarvesterDriveStart(mIntake, 0),

            new AutoSwerveAim(s_Swerve, s_Vision).until(() -> isFinished()),

            new AutoShooterAimAtTarget(mLinearActuator, s_Vision, methods),
            new WaitCommand(0.5),
            new AutoHarvesterDriveStart(mIntake, -0.6),
            new WaitCommand(0.5),
            new AutoHarvesterDriveStart(mIntake, 0)
     




            /*

            new AutoWheels(mWheels, 0.25),   
            new WaitCommand(1),
            new AutoHarvesterDriveStart(mIntake, -0.6),
            new WaitCommand(0.5),

            new ParallelCommandGroup(
                new SequentialCommandGroup(
                   new WaitCommand(0.1),
                    swerveControllerCommand 
                ),
                new AutoWheels(mWheels, 0.3),
                new AutoShooterAimAtTarget(mLinearActuator, s_Vision, methods),
                new WristToggle(mWrist, methods),
                new AutoHarvesterDriveStart(mIntake, 0.6)
            ),
            new StopRobotAutonomous(s_Swerve),
            new AutoHarvesterDriveStart(mIntake, 0),

            new WristToggle(mWrist, methods),
            new AutoShooterAimAtTarget(mLinearActuator, s_Vision, methods),
            new AutoHarvesterDriveStart(mIntake, -0.6),
            new WaitCommand(1)
            */

            );
    }
}