/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commonmethods.CommonMethodExtensions;
import frc.robot.subsystems.*;


public class AutoShooterAimAtTargetCorner extends Command
{
    private final LinearActuator mShooter;
    private final PhotonVision mVision;
    private final CommonMethodExtensions methods;        
    PIDController controller = new PIDController(0.1, 0, 0);

    
    

    public AutoShooterAimAtTargetCorner(LinearActuator subsystem, PhotonVision subsystem2, CommonMethodExtensions methods)
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
        
    }

    @Override
    public void end(boolean interrupted)
    {
        mShooter.setPercentOutput(0);
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
