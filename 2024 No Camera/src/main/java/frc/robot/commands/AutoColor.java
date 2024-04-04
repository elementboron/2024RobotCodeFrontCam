package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;

public class AutoColor extends Command {
    public final Blinkin mBlinkin;

    public AutoColor(Blinkin subsystem) {
        mBlinkin = subsystem;
        addRequirements(subsystem);
    }
    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        if(mBlinkin.getRingDetected(true)) {
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
