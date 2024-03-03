/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.subsystems.*;


public class ShooterToSetpoint extends Command
{
    private final LinearActuator mShooter;
    private final CommonMethodExtensions method;
    double desiredPose;
    
    

    public ShooterToSetpoint(LinearActuator subsystem, CommonMethodExtensions method, double desiredPose)
    {
        mShooter = subsystem;
        this.method = method;
        this.desiredPose = desiredPose;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        desiredPose = mShooter.getDesiredAngle();
        mShooter.MoveToSetPoint(desiredPose);
    }

    @Override
    public boolean isFinished() 
    {
        if(method.Deadband(mShooter.getPosition(), desiredPose,2)){
            return true;
        } else {
            return false;
        }
        
    }
}
