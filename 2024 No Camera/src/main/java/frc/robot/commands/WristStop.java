/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class WristStop extends Command
{
    private final IntakeWrist mWrist;
    

    public WristStop(IntakeWrist subsystem)
    {
        mWrist = subsystem;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {
        mWrist.setPercentOutput(0);
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
