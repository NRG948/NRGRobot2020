package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;

public class WaitForBallReady extends CommandBase {
  private Feeder feeder;
  /**
   * Creates a new WaitForBallReady.
   */
  public WaitForBallReady(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return feeder.isBallInShootingPosition();
  }
}
