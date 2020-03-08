package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class SetHoodPosition extends CommandBase {
  private Hood hood;
  private double desiredPosition;
  private double currentPosition;
  private final static double TOLERANCE = 2;

  /**
   * Creates a new SetHoodPosition.
   */
  public SetHoodPosition(Hood hood, double position) {
    this.hood = hood;
    this.desiredPosition = position;
    addRequirements(hood);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentPosition = hood.getPosition();
    if (desiredPosition < currentPosition) {
      hood.rawHood(-0.25);
    } else {
      hood.rawHood(0.25);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.hoodEnd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(hood.getPosition() - desiredPosition) < TOLERANCE) {
      hood.hoodEnd();
      return true;
    } else {
      return false;
    }
  }
}
