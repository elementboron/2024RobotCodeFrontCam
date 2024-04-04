package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoSwerveAimOther extends Command {    
    private Swerve s_Swerve; 
    private PhotonVision mVision;   
    PIDController controller = new PIDController(0.01,0,0);



    public AutoSwerveAimOther(Swerve s_Swerve, PhotonVision mVision) {
        this.s_Swerve = s_Swerve;
        this.mVision = mVision;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {

            if(mVision.IsabellasGate())
            {
                var result = PhotonVision.camera.getLatestResult();

                //PhotonTrackedTarget target =  mVision.IsabellaTargeter();
                double speed = controller.calculate(mVision.IsabellaAngle(), 0);
               s_Swerve.aprilDrive(
            new Translation2d(0,0), 
            speed * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
            );   
            } else {
                s_Swerve.drive(new Translation2d(0,0), 0, false, true);
            }
            

            
    }

    @Override
    public boolean isFinished() {
        {
            return false;
        }
    }
}