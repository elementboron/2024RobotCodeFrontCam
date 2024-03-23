package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public static SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    PhotonCamera camera = PhotonVision.camera;
    PhotonCamera backCamera = PhotonVision.backCamera;
    PhotonVision mVision = new PhotonVision();

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());
        
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::plannerDrive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    6, // Max module speed, in m/s
                    0.267, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> false,
            this // Reference to this subsystem to set requirements
    );
   
}



    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }   

    public void turnOnly(double rotation) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                new ChassisSpeeds(
                                    0, 
                                    0, 
                                    rotation * 10)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }   

    
 public ChassisSpeeds getChassisSpeeds()
 {
    return Constants.Swerve.swerveKinematics.toChassisSpeeds(this.getModuleStates());
 }

 

     public void plannerDrive(ChassisSpeeds chassis) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassis);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);        
    }
    }
    public void noteDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    if(mVision.IsabellasGateBack())
    {        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        if(mVision.IsabellasGateBack())
        {
          for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }   
        }else {
            drive(new Translation2d(0,0), 0, false, true);
        }
        
    } else {
        drive(translation, 0, false, true);
    }
    }


    public void aprilDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    if(mVision.IsabellasGate())
    {        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        if(camera.getLatestResult().hasTargets())
        {
          for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }   
        }else {
            drive(new Translation2d(0,0), 0, false, true);
        }
        
    } else {
        drive(new Translation2d(0,0), 0, false, true);
    }
    }

    public void AimAtTargetDrive(double translationVal, double strafeVal, BooleanSupplier robotCentricSup, double rotationVal)
    {
        if(mVision.IsabellasGate()){
            PhotonTrackedTarget target = mVision.IsabellaTargeter();
            PIDController controller = new PIDController(0.01,0,0);
            double speed = controller.calculate(target.getYaw(), 0);

            aprilDrive(
            new Translation2d(-translationVal, -strafeVal).times(Constants.Swerve.maxSpeed), 
            speed * Constants.Swerve.maxAngularVelocity, 
            false, 
            true
        );
        } else {
            drive(new Translation2d(translationVal, strafeVal), rotationVal, !robotCentricSup.getAsBoolean(), true);
        }
    }

    public void AimAtNoteDrive(double translationVal, double strafeVal, BooleanSupplier robotCentricSup, double rotationVal)
    {
        if(mVision.IsabellasGateBack()){
            var result = backCamera.getLatestResult();
            PhotonTrackedTarget target = result.getBestTarget();
            PIDController controllerNote = new PIDController(0.0075,0,0.00000000000000);
            double speed = controllerNote.calculate(target.getYaw(), 0);


            noteDrive(
            new Translation2d(-translationVal, -strafeVal).times(Constants.Swerve.maxSpeed), 
            speed * Constants.Swerve.maxAngularVelocity, 
            true, 
            true
            );
        } else {
            drive(new Translation2d(translationVal, strafeVal), rotationVal, !robotCentricSup.getAsBoolean(), true);
        }   
    }

    
    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }


    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetOdometry(Pose2d pose) {
        // //in case of odomoter problems check this first for debug
             swerveOdometry.resetPosition(
                Rotation2d.fromDegrees(getHeadingPath()),
                getModulePositions(),
                pose);
         }

         public double getHeadingPath() {
            return -Math.IEEEremainder(gyro.getAngle(), 360);
        }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public Pose2d resetPose(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(),new Pose2d(getPose().getTranslation(), heading) );
        return swerveOdometry.getPoseMeters();

    }

    public void OneEightyGyro(double angle)
    {
        gyro.setYaw(angle);
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    
    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        SmartDashboard.putNumber("Heading", swerveOdometry.getPoseMeters().getRotation().getDegrees());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}