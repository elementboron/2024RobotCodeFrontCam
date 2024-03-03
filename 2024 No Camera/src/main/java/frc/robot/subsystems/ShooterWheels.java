package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class ShooterWheels extends SubsystemBase {

  public static CANSparkMax mBottomRightMotor = new CANSparkMax(Constants.Motors.bottomRightShooterID, MotorType.kBrushless);
  public static CANSparkMax mTopRightMotor = new CANSparkMax(Constants.Motors.topRightShooterID, MotorType.kBrushless);
  public static CANSparkMax mBottomLeftMotor = new CANSparkMax(Constants.Motors.bottomLeftShooterID, MotorType.kBrushless);
  public static CANSparkMax mTopLeftMotor = new CANSparkMax(Constants.Motors.topLeftShooterID, MotorType.kBrushless);
  public static double wheelSpeed = 0.3;

  public ShooterWheels() {

  }
  
  public double interpolatingPosition(Double distance)
  {
      Double[] valuesArray = {
      Double.valueOf(0), Double.valueOf(.175),
      Double.valueOf(1), Double.valueOf(0.2),
      Double.valueOf(1.5), Double.valueOf(0.3),
      Double.valueOf(5), Double.valueOf(0.3)
      };
    
    InterpolatingDoubleTreeMap treeMap = new InterpolatingDoubleTreeMap();
    treeMap.put(valuesArray[0], valuesArray[1]);
    treeMap.put(valuesArray[2], valuesArray[3]);
    treeMap.put(valuesArray[4], valuesArray[5]);
    treeMap.put(valuesArray[6], valuesArray[7]);
    return treeMap.get(distance);    
  }

  @Override
  public void periodic() {
  }

  public void setPercentOutput(double rightPercentOutput, double leftPercentOutput) {

    mTopRightMotor.set(rightPercentOutput);
    mBottomRightMotor.set(rightPercentOutput);
    mTopLeftMotor.set(leftPercentOutput);
    mBottomLeftMotor.set(leftPercentOutput);
  }

  public void Config()
  {
        mTopRightMotor.restoreFactoryDefaults();
    mBottomRightMotor.restoreFactoryDefaults();
    mTopLeftMotor.restoreFactoryDefaults();
    mBottomLeftMotor.restoreFactoryDefaults();
    mTopRightMotor.setInverted(false);
    mBottomRightMotor.setInverted(true);
    mTopLeftMotor.setInverted(true);
    mBottomLeftMotor.setInverted(false);
    mTopRightMotor.setSmartCurrentLimit(40);
    mBottomRightMotor.setSmartCurrentLimit(40);
    mTopLeftMotor.setSmartCurrentLimit(40);
    mBottomLeftMotor.setSmartCurrentLimit(40);
    SmartDashboard.putNumber("Shooter Wheels Speed", wheelSpeed);
  }

  public double getDesiredSpeed() {
    return SmartDashboard.getNumber("Shooter Wheels Speed", wheelSpeed);
  }

  public double getCurrentSpeed()
  {
    RelativeEncoder encoder = mTopRightMotor.getEncoder();
    return encoder.getVelocity();
  }

  public void setTopBottom(double topPercent, double bottomPercent) {
    mTopRightMotor.set(topPercent);
    mBottomRightMotor.set(bottomPercent);
    mTopLeftMotor.set(topPercent);
    mBottomLeftMotor.set(bottomPercent);
  }

  public void DashboardNumbers() {
    SmartDashboard.putNumber("Wheel Speeds", getCurrentSpeed());
  }

  public boolean FastEnough(double desiredWheelSpeed)
  {
    if(getCurrentSpeed() >= desiredWheelSpeed * 5676){
      return true;
    } else {
      return false;
    }
  }
}