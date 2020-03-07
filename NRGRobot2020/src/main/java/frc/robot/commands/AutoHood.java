package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.utilities.Logger;

/**
 * Enables or disables automatic Shooter Hood positioning based on the distance
 * from the target that is determined via the Limelight camera.
 */
public class AutoHood extends CommandBase {
  private final Hood hood;
  private final boolean enable;

  /**
   * Creates a new AutoHood command.
   */
  public AutoHood(Hood hood, boolean enable) {
    this.hood = hood;
    this.enable = enable;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    Logger.commandInit(this, "enable: " + enable);
    if(enable){
      hood.enableAutoHood();
    } else {
      hood.hoodEnd();
    }
  }

  @Override
  public void execute() {
    // Hood.periodic() does the work to maintain correct Hood angle.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this);

  }

  // This command ends immediately because the real work is done in the Hood subsystem.
  @Override
  public boolean isFinished() {
    return true;
  }
}
