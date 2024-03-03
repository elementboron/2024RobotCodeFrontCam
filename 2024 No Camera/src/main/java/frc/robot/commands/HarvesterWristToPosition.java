/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.commonmethods.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class HarvesterWristToPosition extends Command
{
    private final IntakeWrist mHarvesterWrist;
    private final CommonMethodExtensions method;
    double desiredPose;

    public HarvesterWristToPosition(IntakeWrist subsystem, double desiredPose, CommonMethodExtensions method)
    {
        mHarvesterWrist = subsystem;
        this.desiredPose = desiredPose;
        this.method = method;
        
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        mHarvesterWrist.ToSetPosition(desiredPose);
    }

    @Override
    public boolean isFinished() 
    {
        if (method.Deadband(mHarvesterWrist.GetPosition(), desiredPose, 1))
        {
            return true;
        } else {
            return false;
        }
        
    }
}
