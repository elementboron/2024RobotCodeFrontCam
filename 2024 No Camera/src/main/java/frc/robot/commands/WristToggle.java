/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.subsystems.*;


public class WristToggle extends Command
{
    private final IntakeWrist mWrist;
    private final CommonMethodExtensions methods;
    public double desiredPose;


    public WristToggle(IntakeWrist subsystem, CommonMethodExtensions subsystem2)
    {
        mWrist = subsystem;
        methods = subsystem2;

        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {

        if (mWrist.GetPosition() > 6)
        {
            desiredPose = 0;
           mWrist.setPercentOutput(-0.3);
        } else if(mWrist.GetPosition() < 1){
            desiredPose = 6.6;
            mWrist.setPercentOutput(0.3);
        }
 
    }

    @Override
    public boolean isFinished() 
    {
        if(methods.Deadband(mWrist.GetPosition(), desiredPose, 2.3)){
            mWrist.setPercentOutput(0);
            return true; 
        } else {
            return false;
        }  
    }
}
