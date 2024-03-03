package frc.robot.commonmethods;

public class CommonMethodExtensions {
    public boolean Deadband(double currentPosition, double targetPosition, double tolerance)
    {
        if((currentPosition > targetPosition - tolerance) && (currentPosition < targetPosition + tolerance))
        {
            return true;
        } else {
            return false;
        }
    }
    
}
