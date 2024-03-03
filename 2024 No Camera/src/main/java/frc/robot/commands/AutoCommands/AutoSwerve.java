package frc.robot.commands.AutoCommands;

import frc.robot.Constants;
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


public class AutoSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier targetLock;
    private BooleanSupplier noteLock;
    private PhotonVision mVision;
    public static boolean atGoal;

    public AutoSwerve(Swerve s_Swerve, PhotonVision mVision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier targetLock, BooleanSupplier noteLock) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.targetLock = targetLock;
        this.noteLock = noteLock;
        this.mVision = mVision;
    }

    @Override
    public void initialize(){
        atGoal = false;
    }

    @Override
    public void execute() {
        PhotonCamera camera = PhotonVision.camera;
        PhotonCamera backCamera = PhotonVision.backCamera;
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        if(camera.isConnected() && camera.getLatestResult().hasTargets() ){
            var result = camera.getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();
            PIDController controller = new PIDController(0.01,0,0);
            double speed = controller.calculate(target.getYaw(), 0);
            
            atGoal = controller.atSetpoint();

            s_Swerve.aprilDrive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            speed * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        
        }
        
    }

    @Override
    public boolean isFinished(){
        return atGoal;
    } 
}