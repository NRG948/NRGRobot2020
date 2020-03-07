package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberPiston;
import frc.robot.subsystems.ClimberPiston.State;
import frc.robot.utilities.Logger;

public class SetClimberPiston extends CommandBase {
  // This command isn't currently being used, but it might in the future.
  private ClimberPiston climberPiston;
  private State state;
  /**
   * Creates a new SetClimberWinch.
   */
  public SetClimberPiston(ClimberPiston climberPiston, State state) {
    this.climberPiston = climberPiston;
    this.state = state;
    addRequirements(climberPiston);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this, state.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberPiston.setState(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}