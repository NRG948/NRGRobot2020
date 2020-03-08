package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberPiston;
import frc.robot.utilities.Logger;

/**
 * A command which toggles (inverts) the state of the climber piston.
 * 
 * That is, if the climber arm is currently stowed, this command will extend it.
 * If the climber arm is currently extended, this command will stow it.
 */
public class ToggleClimberPiston extends CommandBase {

  private final ClimberPiston climberPiston;

  /**
   * Creates a new ToggleClimberPiston command.
   */
  public ToggleClimberPiston(ClimberPiston climberPiston) {
    this.climberPiston = climberPiston;
    addRequirements(climberPiston);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberPiston.toggleState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this, interrupted, climberPiston.getState().toString());
  }

  // Returns true when the command should end, which is immediately.
  @Override
  public boolean isFinished() {
    return true;
  }
}
