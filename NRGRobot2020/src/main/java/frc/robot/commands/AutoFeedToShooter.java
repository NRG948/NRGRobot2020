package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.NRGPreferences;

public class AutoFeedToShooter extends CommandBase {
  private Acquirer acquirer;
  private Feeder feeder;

  private double acquirerPower;
  private double feederPower;
  private boolean hasBallBeenShot;
  /**
   * Creates a new AutoFeedToShooter.
   */
  public AutoFeedToShooter(Acquirer acquirer, Feeder feeder) {
    this.acquirer = acquirer;
    this.feeder = feeder;
    addRequirements(feeder, acquirer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    acquirerPower = NRGPreferences.ACQUIRER_POWER.getValue();
    feederPower = NRGPreferences.FEEDER_POWER.getValue();
    hasBallBeenShot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    acquirer.rawAcquirer(acquirerPower);
    feeder.rawFeeder(feederPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when current ball has been shot and a new ball is ready.
  @Override
  public boolean isFinished() {
    if(feeder.isBallInShootingPosition()) {
      return hasBallBeenShot;
    } else {
      hasBallBeenShot = true;
    }
    return false;
  }
}