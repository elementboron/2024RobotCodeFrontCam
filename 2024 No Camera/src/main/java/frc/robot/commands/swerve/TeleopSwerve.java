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


public class TeleopSwerve extends Command {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier targetLock;
    private BooleanSupplier noteLock;
    private PhotonVision mVision;
    private IntakeWrist mWrist;

    public TeleopSwerve(Swerve s_Swerve, PhotonVision mVision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier targetLock, BooleanSupplier noteLock, IntakeWrist mWrist) {
        this.s_Swerve = s_Swerve;
        this.mVision = mVision;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.targetLock = targetLock;
        this.noteLock = noteLock;
        this.mWrist = mWrist;
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
        if(targetLock.getAsBoolean() && mVision.IsabellasGate() ){
            s_Swerve.AimAtTargetDrive(translationVal, strafeVal, robotCentricSup, rotationVal);
            /*
            PhotonTrackedTarget target = mVision.IsabellaTargeter();
            PIDController controller = new PIDController(0.01,0,0);
            double speed = controller.calculate(target.getYaw(), 0);

            s_Swerve.aprilDrive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            speed * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
        );
        */
        } else if (noteLock.getAsBoolean() && mVision.IsabellasGateBack()){
            s_Swerve.AimAtNoteDrive(translationVal, strafeVal, robotCentricSup, rotationVal);
           /* var result = backCamera.getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();
            PIDController controllerNote = new PIDController(0.0075,0,0.00000000000000);
            double speed = controllerNote.calculate(target.getYaw(), 0);


            s_Swerve.noteDrive(
            new Translation2d(-translationVal, -strafeVal).times(Constants.Swerve.maxSpeed), 
            speed * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
            );
            */
            
        } else {
            if(mWrist.GetPosition() < 4.5 ){
                s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true
            ); 
            } else {
                s_Swerve.drive(
                new Translation2d(-translationVal, -strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                false, 
                true
            ); 
            }
            
        }
    }
}