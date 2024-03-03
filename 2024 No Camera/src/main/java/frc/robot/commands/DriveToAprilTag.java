/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.subsystems.*;


public class DriveToAprilTag extends Command
{
    private final PhotonVision s_PhotonVision;
    private final CommonMethodExtensions methods;
    private final Swerve s_Swerve;    
    PhotonTrackedTarget target;
    

    public DriveToAprilTag(Swerve subsystem1, PhotonVision subsystem2, CommonMethodExtensions subsystem3)
    {
        s_Swerve = subsystem1;
        s_PhotonVision = subsystem2;
        methods = subsystem3;
        
        addRequirements(subsystem1, subsystem2);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {     

        PhotonCamera camera = PhotonVision.camera;
        var result = camera.getLatestResult();

        if(result.hasTargets()){
            PhotonTrackedTarget target = result.getBestTarget();
    
            PIDController controllerX = new PIDController(1,0,0);
            PIDController controllerY = new PIDController(1,0,0);
            PIDController controllerRot = new PIDController(1,0,0);

            double xSpeed = controllerX.calculate(target.getBestCameraToTarget().getX(), 1);
            double zSpeed = controllerX.calculate(target.getBestCameraToTarget().getZ(), 2);
            double rotSpeed = controllerX.calculate(target.getYaw(), 0);
            s_Swerve.drive(new Translation2d(xSpeed/5, zSpeed/5), rotSpeed, false, true);
        }
        
        
    }

    @Override
    public boolean isFinished() 
    {
        if(methods.Deadband(target.getBestCameraToTarget().getX(),1, .2)
            && methods.Deadband(target.getBestCameraToTarget().getZ(), 2, .2)
            && methods.Deadband(target.getBestCameraToTarget().getRotation().getAngle(), 0, 0.1)) 
            {
                return true;
            } else {
                return false;
            }
    }
}
