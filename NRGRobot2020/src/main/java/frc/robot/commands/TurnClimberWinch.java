package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberWinch;
import frc.robot.utilities.Logger;

public class TurnClimberWinch extends CommandBase {
  private ClimberWinch climberWinch;
  private double maxPower;
  /**
   * Creates a new ManualClimberWinch.
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
    climberWinch.rawClimb(maxPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberWinch.stopMotor();
    Logger.commandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
