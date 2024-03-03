/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.State;
import frc.robot.subsystems.*;


public class StateJoystick extends Command
{
    private final PhotonVision s_PhotonVision;
    
    

    public StateJoystick(PhotonVision subsystem)
    {
        s_PhotonVision = subsystem;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
        Robot.botState = State.Joystick;
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
