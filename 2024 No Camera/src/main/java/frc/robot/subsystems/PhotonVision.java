// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.photonvision.*;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {

  public static final PhotonCamera camera = new PhotonCamera("ArduCamFront2");
  public static final PhotonCamera backCamera = new PhotonCamera("Pi2Back");

  final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  //Transform3d robotToCam = new Transform3d(new Translation3d(0.32, 0.085, 0.23), new Rotation3d(0, Math.toRadians(140),0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.



  public static List<PhotonTrackedTarget> lastSeenTarget;

  public static double lastGoodTarget;



  /** Creates a new Limelight. */
  public PhotonVision() {   
  }

  public boolean HasTargets()
  {
    var result = camera.getLatestResult();
    return result.hasTargets();

  }

  public void ShooterCamOn()
  {
    backCamera.setDriverMode(true);
    camera.setDriverMode(false);
  }

  public void NoteCamOn()
  {
    backCamera.setDriverMode(false);
    camera.setDriverMode(true);
  }


  public void UpdateSmartDashboard()
  {
    var result = camera.getLatestResult();
    SmartDashboard.putBoolean("Sees Target?", result.hasTargets());
    //SmartDashboard.putNumber("Photon Yaw", result.getBestTarget().getYaw());
  }

  public List<PhotonTrackedTarget> SetMostRecentAprilTag()
  {
    var result = camera.getLatestResult();
    lastSeenTarget = result.getTargets();
    

    return lastSeenTarget;
  }

  public double getDistanceFromTarget()
  {
    if(IsabellasGate())
    {
    var result = camera.getLatestResult();
    PhotonTrackedTarget target = IsabellaTargeter();
    if( target == null )
    {
      return 0;
    }   
    double distX = target.getBestCameraToTarget().getX();    
    double distY = target.getBestCameraToTarget().getY();
    lastGoodTarget = Math.abs(Math.sqrt((distX*distX) + (distY*distY)));
    return lastGoodTarget;
    } else {
      return 0;
    }
    
  }


  public double getAngleToNote()
  {
    var result = backCamera.getLatestResult();
    return result.getBestTarget().getYaw();
  }

  public BooleanSupplier hasTargetSupplier()
  {
    BooleanSupplier supplier = () -> camera.getLatestResult().hasTargets();
    return supplier;
  }

  public boolean IsabellasGate()
  {
    if(camera.isConnected())
    {
      var result = camera.getLatestResult();

      if(result.hasTargets())
      {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

    public boolean IsabellasGateBack()
  {
    if(backCamera.isConnected())
    {
      var result = backCamera.getLatestResult();

      if(result.hasTargets())
      {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }

  public PhotonTrackedTarget IsabellaTargeter()
  {
    var result = camera.getLatestResult();
    if(IsabellasGate()) {
    for (PhotonTrackedTarget i : result.getTargets())
    {
      if(i.getFiducialId() == 4 || i.getFiducialId() == 7)
      {
        return i;
      }
    }
    return result.getBestTarget();
  } else {
    return result.getBestTarget();
  }
  }

  @Override
  public void periodic() {
  }
}
