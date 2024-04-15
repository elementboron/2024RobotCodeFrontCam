/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class TeleopHarvesterIntake extends Command
{
    private final IntakeDrive m_harvesterDrive;
    IntakeWrist mWrist;
    double speed;
    
    

    public TeleopHarvesterIntake(IntakeDrive subsystem, double mSpeed, IntakeWrist mWrist)
    {
        m_harvesterDrive = subsystem;
        speed = mSpeed;
        this.mWrist = mWrist;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  

        if(mWrist.GetPosition() < 2)
        {
            m_harvesterDrive.setPercentOutput(0);
        } else {
            m_harvesterDrive.setPercentOutput(speed);
        }
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
