package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeWrist extends SubsystemBase {
  private final TalonFX mHarvesterMotor = Robot.mWristMotor;

  public IntakeWrist() {}

  public void ToSetPosition(double desiredPosition){

    //clamp
    if(desiredPosition < Constants.Motors.intakeWristMin){
      desiredPosition = Constants.Motors.intakeWristMin;
    } else if (desiredPosition > Constants.Motors.intakeWristMax){
      desiredPosition = Constants.Motors.intakeWristMax;
    }


      PIDController controller = new PIDController(0.03,0,0);
      double speed = controller.calculate(mHarvesterMotor.getPosition().getValueAsDouble(), desiredPosition);
      mHarvesterMotor.set(speed);
  }


  public void Config()
  {
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    SoftwareLimitSwitchConfigs softLimits = toConfigure.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = Constants.Motors.intakeWristMax;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = Constants.Motors.intakeWristMin;
    mHarvesterMotor.getConfigurator().apply(softLimits);
   
  }

  public double GetPosition() {
    return mHarvesterMotor.getPosition().getValueAsDouble();

  }
  
  @Override
  public void periodic() {
  }

  public void DashBoardNumbers(){
    SmartDashboard.putNumber("Havester Wrist Angle", mHarvesterMotor.getPosition().getValueAsDouble());

  }

  public void setPercentOutput(double motorPercentOutput) {
    mHarvesterMotor.set(motorPercentOutput);
  }
}