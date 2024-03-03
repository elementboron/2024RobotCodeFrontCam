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


public class AutoSwerveAimAtNote extends Command {    
    private Swerve s_Swerve;    
    double rotation;
    PIDController controller = new PIDController(0.01,0,0);



    public AutoSwerveAimAtNote(Swerve s_Swerve, PhotonVision mVision, double rotation) {
        this.s_Swerve = s_Swerve;
        this.rotation = rotation;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
            PhotonCamera camera = PhotonVision.backCamera;

            if(camera.isConnected() && camera.getLatestResult().hasTargets()) {
            var result = camera.getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();
            double speed = controller.calculate(target.getYaw(), 0);
            controller.setTolerance(3);

            s_Swerve.aprilDrive(
            new Translation2d(0,0), 
            speed * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
            );  
            } else 
            {
                s_Swerve.drive(new Translation2d(0,0), 0, true, true);
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