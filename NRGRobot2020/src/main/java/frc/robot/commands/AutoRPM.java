package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.Logger;

public class AutoRPM extends CommandBase {
  private final ShooterRPM shooterRPM;
  private final boolean enable;
  /**
   * Creates a new AutoRPM.
   */
  public AutoRPM(ShooterRPM shooterRPM, boolean enable) {
    this.shooterRPM = shooterRPM;
    this.enable = enable;
    addRequirements(shooterRPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterRPM.enableAutoRPM(enable);
    shooterRPM.setGoalRPM(1000); // Get the shooter wheel started
    Logger.commandInit(this, "enable: " + enable);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
