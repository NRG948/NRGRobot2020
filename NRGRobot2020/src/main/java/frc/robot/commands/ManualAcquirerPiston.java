package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.AcquirerPiston;
import frc.robot.subsystems.AcquirerPiston.State;

public class ManualAcquirerPiston extends CommandBase {
  private State state;
  private AcquirerPiston acquirerPiston;
  /**
   * Creates a new ManualAcquirerPiston.
   */
  public ManualAcquirerPiston(AcquirerPiston acquirerPiston, JoystickButton joystickButton) {
    this.acquirerPiston = acquirerPiston;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    acquirerPiston.setState();
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}