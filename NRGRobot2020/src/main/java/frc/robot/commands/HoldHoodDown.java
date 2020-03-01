package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.utilities.Logger;

public class HoldHoodDown extends CommandBase {

  private final Hood hood;
  private double positionToRestore;

  /**
   * Creates a new HoldAcquirerDown.
   */
  public HoldHoodDown(Hood hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this);
    positionToRestore = hood.getPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (hood.getPosition() > 2) {
      hood.rawHood(-0.25); // lower the hood
    } else {
      hood.rawHood(0);
    }
  }
  
  // Manual commands never end, they only get interrupted.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    new SetHoodPosition(hood, positionToRestore).schedule();
    Logger.commandEnd(this, String.format("returning hood to pos: %3.1f", positionToRestore));
  }
}
