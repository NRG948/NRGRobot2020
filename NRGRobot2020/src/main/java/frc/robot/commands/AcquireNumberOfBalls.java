package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.BallCounter;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;

public class AcquireNumberOfBalls extends CommandBase {

  private final Acquirer acquirer;
  private final BallCounter ballCounter;

  private double acquirerPower;
  private int numberOfBalls;
  private int targetCount;
  private boolean relativeCount;

  /**
   * Creates a new AcquirerForSeconds.
   */
  public AcquireNumberOfBalls(Acquirer acquirer, BallCounter ballCounter) {
    this.acquirer = acquirer;
    this.ballCounter = ballCounter;
    /* BallCounter is also used by AutoFeeder, therefore we do not add it to requirements.*/
    addRequirements(acquirer);
  }

  public AcquireNumberOfBalls withAbsoluteCount(int numberOfBalls) {
    this.relativeCount = false;
    this.numberOfBalls = numberOfBalls;
    return this;
  }

  public AcquireNumberOfBalls withRelativeCount(int numberOfBalls) {
    this.relativeCount = true;
    this.numberOfBalls = numberOfBalls;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    acquirerPower = NRGPreferences.ACQUIRER_POWER.getValue();
    targetCount = numberOfBalls;
    if (relativeCount) {
      targetCount += ballCounter.getBallCount();
    }
    Logger.commandInit(this, targetCount + " balls");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    acquirer.rawAcquirer(acquirerPower);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (targetCount <= ballCounter.getBallCount());
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    acquirer.stop();
    Logger.commandEnd(this);
  }
}
