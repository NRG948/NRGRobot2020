package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberWinch;
import frc.robot.utilities.Logger;

/**
 * A command which can run the climber winch in either direction.
 * 
 * Caution: the winch uses a ratcheting mechanism, so it can only safely
 * be run in the "climb up" direction (positive motor power).
 */
public class TurnClimberWinch extends CommandBase {
  private ClimberWinch climberWinch;
  private double maxPower;

  /**
   * Creates a new ManualClimberWinch command.
   */
  public TurnClimberWinch(ClimberWinch climberWinch) {
    this.climberWinch = climberWinch;
    addRequirements(climberWinch);
  }

  public TurnClimberWinch withMaxPower(double maxPower) {
    this.maxPower = maxPower;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this, "maxPower: " + maxPower);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberWinch.rawClimbUp(maxPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberWinch.stopMotor();
    Logger.commandEnd(this, interrupted);
  }

  // This command only ends when it is interrupted. Invoke using .whenHeld(...).
  @Override
  public boolean isFinished() {
    return false;
  }
}
