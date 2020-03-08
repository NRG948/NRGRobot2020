package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberPiston;
import frc.robot.utilities.Logger;

public class ToggleClimberPiston extends CommandBase {

  private final ClimberPiston climberPiston;

  /**
   * Creates a new ToggleAcquirerPiston command.
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
    Logger.commandEnd(this);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
