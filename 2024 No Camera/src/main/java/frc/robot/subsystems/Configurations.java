package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Configurations extends SubsystemBase {

  public static int centerSpeakerID;
  public static int offsetSpeakerID;
  public static int ampID;
  public static int farLoadID;
  public static int closeLoadID;
  public static enum Color {Red, Blue};
  public static int[] array;


  public int[] Configurations(Color color) {

    if(color == Color.Blue)
    {
      centerSpeakerID = 7;
      offsetSpeakerID = 8;
      ampID = 6;
      farLoadID = 2;
      closeLoadID = 1;
      array = new int[] {centerSpeakerID, offsetSpeakerID, ampID, farLoadID, closeLoadID};
     return array; 
    } else {
      centerSpeakerID = 4;
      offsetSpeakerID =3;
      ampID = 5;
      farLoadID = 9;
      closeLoadID = 10;
      array = new int[] {centerSpeakerID, offsetSpeakerID, ampID, farLoadID, closeLoadID};
      return array;
    }

  }


}