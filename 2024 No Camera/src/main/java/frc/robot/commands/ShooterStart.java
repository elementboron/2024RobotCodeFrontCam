package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterWheels;

public class ShooterStart extends Command {
  private final ShooterWheels mShooter;
  private final PhotonVision mVision;

  boolean active;
  

  public ShooterStart(ShooterWheels subsystem1, PhotonVision subsystem2) {
    mShooter = subsystem1;
    mVision = subsystem2;

    addRequirements(subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //mShooter.setPercentOutput(mRightPercentOutput, mLeftPercentOutput, mfeeder);
    if(mVision.IsabellasGate()){
      double distance = mVision.getDistanceFromTarget();
      mShooter.setPercentOutput(mShooter.interpolatingPosition(distance), mShooter.interpolatingPosition(distance) * 0.75);
    } else {
      mShooter.setPercentOutput(0, 0);
    }
    } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}