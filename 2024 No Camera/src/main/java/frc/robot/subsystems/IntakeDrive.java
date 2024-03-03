package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeDrive extends SubsystemBase {
  private final CANSparkMax mHarvesterMotor = Robot.mHarvesterMotor;

  public IntakeDrive() {
    mHarvesterMotor.restoreFactoryDefaults();
    mHarvesterMotor.setSmartCurrentLimit(40);
  }



  @Override
  public void periodic() {
  }

  public void setPercentOutput(double motorPercentOutput) {
    mHarvesterMotor.set(motorPercentOutput);
  }
}