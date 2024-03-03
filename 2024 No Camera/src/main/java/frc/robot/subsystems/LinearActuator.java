package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commonmethods.CommonMethodExtensions;
public class LinearActuator extends SubsystemBase {
  
  TalonFX mLinearActuator = Robot.mLinearActuatorMotor;
  CommonMethodExtensions methods = new CommonMethodExtensions();
  PhotonCamera camera = PhotonVision.camera;

  public static double desiredActuator = 0.0;





  public void Config()
  {
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    SoftwareLimitSwitchConfigs softLimits = toConfigure.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = Constants.Motors.shooterMax;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = Constants.Motors.shooterMin;
    mLinearActuator.getConfigurator().apply(softLimits);

    SmartDashboard.putNumber("Desired Actuator Angle", desiredActuator);

  }

  public double CalculateDesiredShooterAngle(double[] botPosition, double[] targetPosition)
  {
    double botX = botPosition[0];
    double botY = botPosition[1];
    double targetX = targetPosition[0];
    double targetY = targetPosition[1];
    double targetH = targetPosition[2];

    double distance = (botY - targetY) / (botX - targetX);
    double slopeDirection = distance/Math.abs(distance);
    distance = distance * slopeDirection;

    double angle;
    angle = Math.atan(targetH/distance);
    angle = angle * (180/Math.PI);

    return angle;
  }

  public void moveToSetPoint(double desiredPose)
  {
    if(desiredPose > Constants.Motors.shooterMax || desiredPose <  Constants.Motors.shooterMin)
    {
      mLinearActuator.set(0);
    } else {
      PIDController controller = new PIDController(0,0,0);
      double speed = controller.calculate(mLinearActuator.getPosition().getValueAsDouble(), desiredPose);
      mLinearActuator.set(speed) ;

    }
  }

  public boolean MoveToSetPoint(double desiredPosition){

    //if(!methods.Deadband(desiredPosition, encoder.getPosition(), 2)){

    if(camera.getLatestResult().hasTargets())
    {
      if(desiredPosition < Constants.Motors.shooterMin)
    {
      desiredPosition = Constants.Motors.shooterMin;
    } else if (desiredPosition > Constants.Motors.shooterMax)
    {
      desiredPosition = Constants.Motors.shooterMax;
    } 

    PIDController controller = new PIDController(0.1,0,0.00001);
    double speed = controller.calculate(mLinearActuator.getPosition().getValueAsDouble(), desiredPosition);
    mLinearActuator.set(speed);
    return controller.atSetpoint();
    } else {
      mLinearActuator.set(0);
      return true;
    }

    
  }
    

  public double regressiveEquation(double distanceToTarget) {
    double actuatorDistance;
    if(distanceToTarget < 2){
      distanceToTarget = 2.000001;
    }

    SmartDashboard.putNumber("Estimated Distance", distanceToTarget);

    if(distanceToTarget > 3.7)
    {
      actuatorDistance = -distanceToTarget-93.4;
    } else {
      actuatorDistance = (((Math.sqrt((4750*(distanceToTarget))-9500))*-1));      
    } 


    SmartDashboard.putNumber("estimated angle", actuatorDistance);

    return actuatorDistance;
  }

  public double interpolatingPosition(Double distance)
  {
    Double[] valuesArray = {
      Double.valueOf(0), Double.valueOf(0),
      Double.valueOf(1), Double.valueOf(0),
      Double.valueOf(1.85), Double.valueOf(-20),
      Double.valueOf(2.23), Double.valueOf(-45),
      Double.valueOf(2.47), Double.valueOf(-52),
      Double.valueOf(2.71), Double.valueOf(-60),
      Double.valueOf(2.96), Double.valueOf(-67),
      Double.valueOf(3.23), Double.valueOf(-85),
      Double.valueOf(3.61), Double.valueOf(-90),
      Double.valueOf(3.7), Double.valueOf(-95),
      Double.valueOf(3.92), Double.valueOf(-100),
      Double.valueOf(4.2), Double.valueOf(-105)
        };
    
    InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();
    treeMap.put(valuesArray[0], valuesArray[1]);
    treeMap.put(valuesArray[2], valuesArray[3]);
    treeMap.put(valuesArray[4], valuesArray[5]);
    treeMap.put(valuesArray[6], valuesArray[7]);
    treeMap.put(valuesArray[8], valuesArray[9]);
    treeMap.put(valuesArray[10], valuesArray[11]);
    treeMap.put(valuesArray[12], valuesArray[13]);
    treeMap.put(valuesArray[14], valuesArray[15]);
    treeMap.put(valuesArray[16], valuesArray[17]);
    treeMap.put(valuesArray[18], valuesArray[19]);
    treeMap.put(valuesArray[20], valuesArray[21]);
    treeMap.put(valuesArray[22], valuesArray[23]);




    SmartDashboard.putNumber("Estimated Actuator Goal", treeMap.get(distance));
    return treeMap.get(distance);    
  }



  public void DashboardNumbers() {
    SmartDashboard.putNumber("Linear Actuator Angle", mLinearActuator.getPosition().getValueAsDouble());
  }

  public double getDesiredAngle() {
    return SmartDashboard.getNumber("Desired Actuator Angle", desiredActuator);
  }

  public void setPercentOutput(double motorPercentOutput) {
    mLinearActuator.set(motorPercentOutput);
  }

  public double getPosition() {
    return mLinearActuator.getPosition().getValueAsDouble();
  }


}  