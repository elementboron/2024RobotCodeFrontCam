package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterWheels;

public class AutoWheelsChecker extends Command {
  private final ShooterWheels mShooter;
  private final PhotonVision mVision;
  double speed;


  boolean active;
  

  public AutoWheelsChecker(ShooterWheels subsystem1, PhotonVision subsystem2, double speed) {
    mShooter = subsystem1;
    mVision = subsystem2;
    this.speed = speed;

    addRequirements(subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   active = mShooter.FastEnough(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return active;
  }
}