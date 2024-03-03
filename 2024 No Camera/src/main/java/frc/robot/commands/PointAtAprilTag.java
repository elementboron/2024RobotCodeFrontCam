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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.*;


public class PointAtAprilTag extends Command
{
    private final PhotonVision s_PhotonVision;
    private final Swerve s_Swerve;
    private final PhotonCamera m_Camera;
    private double rotSpeed;
    
    

    public PointAtAprilTag(Swerve subsystem1, PhotonVision subsystem2, PhotonCamera camera)
    {
        s_PhotonVision = subsystem2;
        s_Swerve = subsystem1;
        m_Camera = camera;

        
        addRequirements(subsystem1, subsystem2);
    }

    @Override
    public void initialize(){
        
    }
    
    @Override
    public void execute() 
    {     
        var result = m_Camera.getLatestResult();
        PhotonTrackedTarget target = result.getBestTarget();
    
        PIDController controllerTheta = new PIDController(1,0, 0.05);
            
        if(s_PhotonVision.HasTargets()){
                rotSpeed = controllerTheta.calculate(target.getYaw(), 0);
            } else {
                rotSpeed = 0;
            }


        SwerveModule[] mSwerveMods = s_Swerve.mSwerveMods;

        SwerveModuleState[] state = Constants.Swerve.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0,rotSpeed));
        SwerveDriveKinematics.desaturateWheelSpeeds(state, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(state[mod.moduleNumber], true);
        }


    }

    @Override
    public boolean isFinished() 
    {
        /*boolean finish = false;
        new WaitCommand(1000);
        finish = true;
        return finish;*/
        return true;
    }
}
