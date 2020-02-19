package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AcquirerPiston;

public class ToggleAcquirerPiston extends CommandBase {

  private AcquirerPiston acquirerPiston;

  /**
   * Creates a new ToggleAcquirerPiston.
   */
  public ToggleAcquirerPiston(AcquirerPiston acquirerPiston) {
    this.acquirerPiston = acquirerPiston;
    addRequirements(acquirerPiston);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      acquirerPiston.toggleState();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
