/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.subsystems.*;


public class AutoShooterAimAtTarget extends Command
{
    private final LinearActuator mShooter;
    private final PhotonVision mVision;
    private final CommonMethodExtensions methods;
    
    

    public AutoShooterAimAtTarget(LinearActuator subsystem, PhotonVision subsystem2, CommonMethodExtensions methods)
    {
        mShooter = subsystem;
        mVision = subsystem2;
        this.methods = methods;
        
        addRequirements(subsystem, subsystem2);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        if(PhotonVision.camera.getLatestResult().hasTargets() && (PhotonVision.camera.getLatestResult().getBestTarget().getFiducialId() == 4 || PhotonVision.camera.getLatestResult().getBestTarget().getFiducialId() == 7)){
        double distance = mVision.getDistanceFromTarget();
        SmartDashboard.putNumber("New Distance", distance);
        mShooter.MoveToSetPoint(mShooter.interpolatingPosition(Double.valueOf(distance)));            
        } else if (PhotonVision.camera.getLatestResult().hasTargets()){
            mShooter.MoveToSetPoint(0);
        } else {
            mShooter.setPercentOutput(0);
        }
    }

    @Override
    public void end(boolean interrupted)
    {
        mShooter.setPercentOutput(0);
    }

    @Override
    public boolean isFinished() 
    {
        if(methods.Deadband(mShooter.getPosition(), mShooter.interpolatingPosition(mVision.getDistanceFromTarget()), 1))
        {
            return true;
        } else {
            return false;
        }
    }
}
