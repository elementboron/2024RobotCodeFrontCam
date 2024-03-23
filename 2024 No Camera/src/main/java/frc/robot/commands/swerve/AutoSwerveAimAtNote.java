package frc.robot.commands.swerve;

import frc.robot.Constants;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class AutoSwerveAimAtNote extends Command {    
    private Swerve s_Swerve; 
    private PhotonVision mVision;   
    PIDController controller = new PIDController(0.01,0,0);



    public AutoSwerveAimAtNote(Swerve s_Swerve, PhotonVision mVision) {
        this.s_Swerve = s_Swerve;
        this.mVision = mVision;
    }

    @Override
    public void execute() {

        if(mVision.IsabellasGateBack()){
            var result = PhotonVision.backCamera.getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();
            PIDController controllerNote = new PIDController(0.0075,0,0.00000000000000);
            double speed = controllerNote.calculate(target.getYaw(), 0);


            s_Swerve.noteDrive(
            new Translation2d(0, 0).times(Constants.Swerve.maxSpeed), 
            speed * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
            ); 
            } else {
                s_Swerve.drive(new Translation2d(0,0), 0, false, true);
            }
            

            
    }

    @Override
    public void end(boolean i)
    {
        s_Swerve.drive(new Translation2d(0,0), 0, false, true);
    }


    @Override
    public boolean isFinished() {
        {
            return false;

            /*

            if(controller.atSetpoint())
            {
                return true;
            } else {
                return false;
            } 
            */
        }
    }
}