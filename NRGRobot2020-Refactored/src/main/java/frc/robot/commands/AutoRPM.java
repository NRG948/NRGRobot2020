package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.Logger;

/**
 * Automatically sets the Shooter wheel RPM based on the distance from the Limelight camera.
 */
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
    Logger.commandInit(this, "enable: " + enable);
    shooterRPM.enableAutoRPM(enable);
    if (enable) {
      shooterRPM.setGoalRPM(2000); // Get the shooter wheel started
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // ShooterRPM.periodic() does the work to maintain correct RPM speed.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
