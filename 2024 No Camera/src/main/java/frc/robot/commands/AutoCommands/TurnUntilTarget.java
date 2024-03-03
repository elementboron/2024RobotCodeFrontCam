/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoCommands;


import org.photonvision.proto.Photon;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.*;


public class TurnUntilTarget extends Command
{
    private final Swerve s_Swerve;
    private final PhotonVision mVision;
    double rotation;
    
    

    public TurnUntilTarget(Swerve subsystem, PhotonVision subsystem2, double rotation)
    {
        s_Swerve = subsystem;
        mVision = subsystem2;
        this.rotation = rotation;
        
        addRequirements(subsystem, subsystem2);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {  
    }

    @Override
    public boolean isFinished() 
    {
        return false;
    }
}
