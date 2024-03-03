package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterWheels;

public class ShooterTuning extends Command {
  private final ShooterWheels mShooter;
  private final double mRightPercentOutput;
  private final double mLeftPercentOutput;
  boolean active;
  

  public ShooterTuning(ShooterWheels subsystem1, double rightPercentOutput, double leftPercentOutput) {
    mShooter = subsystem1;
    mRightPercentOutput = rightPercentOutput;
    mLeftPercentOutput = leftPercentOutput;
    addRequirements(subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //mShooter.setPercentOutput(mRightPercentOutput, mLeftPercentOutput, mfeeder);
    mShooter.setPercentOutput(mRightPercentOutput, mLeftPercentOutput);
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