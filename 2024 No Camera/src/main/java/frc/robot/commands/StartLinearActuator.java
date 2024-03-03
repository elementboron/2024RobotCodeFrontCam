/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class StartLinearActuator extends Command
{
    private final LinearActuator mLinearActuator;
    private final PhotonVision mVision;
    DoubleSupplier speedX;
    DoubleSupplier speedY;

    
    
    

    public StartLinearActuator(LinearActuator subsystem, PhotonVision subsystem2, DoubleSupplier speedX, DoubleSupplier speedY)
    {
        mLinearActuator = subsystem;
        mVision = subsystem2;
        
        this.speedX = speedX;
        this.speedY = speedY;
        
        addRequirements(subsystem, subsystem2);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {   
        mLinearActuator.setPercentOutput((speedX.getAsDouble() - speedY.getAsDouble()));
    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
