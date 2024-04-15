package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ShooterWheels;

public class AutoColor extends Command {
    public final Blinkin mBlinkin;
    public final ShooterWheels m_Wheels;

    public AutoColor(Blinkin subsystem, ShooterWheels subsystem2) {
        mBlinkin = subsystem;
        m_Wheels = subsystem2;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(m_Wheels.WheelSpeedCheck()) {
            mBlinkin.SetSpeed(.15);
        } else if(mBlinkin.getRingDetected(true)) {
            mBlinkin.SetSpeed(.33);
        } else {
            mBlinkin.SetSpeed(.13);
        }
    
    }
    

    @Override
    public boolean isFinished() {
        return true;
    }
}
