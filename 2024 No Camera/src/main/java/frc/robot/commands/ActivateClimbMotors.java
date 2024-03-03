package frc.robot.commands;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class ActivateClimbMotors extends Command
{
    private final Climber mClimber;
    DoubleSupplier speedX;
    DoubleSupplier speedY;

    
    
    

    public ActivateClimbMotors(Climber subsystem, DoubleSupplier speedX, DoubleSupplier speedY)
    {
        mClimber = subsystem;
        
        this.speedX = speedX;
        this.speedY = speedY;
        
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){}
    
    @Override
    public void execute() 
    {   
        /*
        if(mClimber.GetPosition() < 380 && mClimber.GetPosition() > 0)
        {
            mClimber.setPercentOutput((speedX.getAsDouble() - speedY.getAsDouble()));
        } else if (mClimber.GetPosition() > 380) {
            mClimber.setPercentOutput(-0.1);
        } else {
            mClimber.setPercentOutput(0.1);
        }
        */
    mClimber.setPercentOutput((speedX.getAsDouble() - speedY.getAsDouble()));

    }

    @Override
    public boolean isFinished() 
    {
        return true;
    }
}
