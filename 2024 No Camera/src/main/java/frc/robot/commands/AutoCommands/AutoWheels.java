package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterWheels;

public class AutoWheels extends Command {
  private final ShooterWheels mShooter;
  double speed;

  boolean active;
  

  public AutoWheels(ShooterWheels subsystem1,  double speed) {
    mShooter = subsystem1;
    this.speed = speed;

    addRequirements(subsystem1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mShooter.setPercentOutput(speed, speed * 0.75);
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