/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class AutoHarvesterDriveStart extends Command
{
    private final IntakeDrive m_harvesterDrive;
    double speed;
    
    

    public AutoHarvesterDriveStart(IntakeDrive subsystem, double mSpeed)
    {
        m_harvesterDrive = subsystem;
        speed = mSpeed;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        m_harvesterDrive.setPercentOutput(speed);
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
