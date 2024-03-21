/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class ShooterCamActivate extends Command
{
    private final PhotonVision s_PhotonVision;
    
    

    public ShooterCamActivate(PhotonVision subsystem)
    {
        s_PhotonVision = subsystem;
        
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        s_PhotonVision.ShooterCamOn();
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
