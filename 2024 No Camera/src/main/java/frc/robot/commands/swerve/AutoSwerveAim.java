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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoSwerveAim extends Command {    
    private Swerve s_Swerve; 
    private PhotonVision mVision;   
    PIDController controller = new PIDController(0.01,0,0);



    public AutoSwerveAim(Swerve s_Swerve, PhotonVision mVision) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
            PhotonCamera camera = PhotonVision.camera;

            if(mVision.IsabellasGate()) {
            
            var result = camera.getLatestResult();
            PhotonTrackedTarget target = mVision.IsabellaTargeter();
            double speed = controller.calculate(target.getYaw(), 0);
            controller.setTolerance(2);

            s_Swerve.aprilDrive(
            new Translation2d(0,0), 
            speed * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
            );  
            } else 
            {
                s_Swerve.drive(new Translation2d(0,0), 1, true, true);
            }
            
            
    }

    @Override
    public boolean isFinished() {
        {
            if(controller.atSetpoint())
            {
                return true;
            } else {
                return false;
            }
        }
    }
}