package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
  private final TalonFX mFrontClimberMotor = new TalonFX(Constants.Motors.frontClimbID);
  private final TalonFX mBackClimberMotor = new TalonFX(Constants.Motors.backClimbID);

  public Climber() {
  }

  public void Config()
  {
    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    SoftwareLimitSwitchConfigs softLimits = toConfigure.SoftwareLimitSwitch;
    softLimits.ForwardSoftLimitEnable = true;
    softLimits.ForwardSoftLimitThreshold = 380;
    softLimits.ReverseSoftLimitEnable = true;
    softLimits.ReverseSoftLimitThreshold = 0;
    mFrontClimberMotor.getConfigurator().apply(softLimits);
    mBackClimberMotor.getConfigurator().apply(softLimits);
  }

  public double GetPosition()
  {
    return mFrontClimberMotor.getPosition().getValueAsDouble();
  }

  public void Dashboard()
  {
    SmartDashboard.putNumber("climber pose", mFrontClimberMotor.getPosition().getValueAsDouble());
  }



  @Override
  public void periodic() {
  }

  public void setPercentOutput(double motorPercentOutput) {
    mFrontClimberMotor.set(motorPercentOutput);
    mBackClimberMotor.set(motorPercentOutput);
  }
}