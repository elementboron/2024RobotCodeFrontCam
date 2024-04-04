package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase{
    
    public static Spark mBlinkin = new Spark(9);
    public static DigitalInput mRingSensor = new DigitalInput(8);
    
    public Blinkin() {

    }

    public void SetSpeed(double speed) {
        mBlinkin.setSafetyEnabled(false);
        mBlinkin.set(speed);
    }
    public boolean getRingDetected(boolean ringDetected) {
        return mRingSensor.get();
    }

    public void SmartDashboard()
    {
        edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("Value", mRingSensor.get());
    }

    @Override
    public void periodic(){

    }
}
