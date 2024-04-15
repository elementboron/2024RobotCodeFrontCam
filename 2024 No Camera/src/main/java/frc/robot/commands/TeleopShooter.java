package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterWheels;

public class TeleopShooter extends Command {
  private final ShooterWheels mShooter;
  private final DoubleSupplier mRightPercentOutput;
  private final DoubleSupplier mLeftPercentOutput;
  boolean active;
  

  public TeleopShooter(ShooterWheels subsystem1, DoubleSupplier rightPercentOutput, DoubleSupplier leftPercentOutput) {
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
    mShooter.setPercentOutput(mRightPercentOutput.getAsDouble(), mLeftPercentOutput.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}