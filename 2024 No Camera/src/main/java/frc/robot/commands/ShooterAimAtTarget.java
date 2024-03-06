/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.subsystems.*;


public class ShooterAimAtTarget extends Command
{
    private final LinearActuator mShooter;
    private final PhotonVision mVision;
    
    

    public ShooterAimAtTarget(LinearActuator subsystem, PhotonVision subsystem2)
    {
        mShooter = subsystem;
        mVision = subsystem2;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        if(mVision.IsabellasGate()){
        double distance = mVision.getDistanceFromTarget();
        SmartDashboard.putNumber("New Distance", distance);
        mShooter.MoveToSetPoint(mShooter.interpolatingPosition(Double.valueOf(distance)));            
        } else {
        mShooter.setPercentOutput(0);
    }
}

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
