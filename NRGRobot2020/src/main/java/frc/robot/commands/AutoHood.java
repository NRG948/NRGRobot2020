package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.utilities.Logger;

/**
 * Automatically sets the Shooter Hood based on the distince from the Limelight
 * camera.
 */
public class AutoHood extends CommandBase {
  private final Hood hood;
  private final boolean enable;

  /**
   * Creates a new AutoHood.
   */
  public AutoHood(Hood hood, boolean enable) {
    this.hood = hood;
    this.enable = enable;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this, "enable: " + enable);
    if(enable){
      hood.enableAutoHood();
    } else {
      hood.hoodEnd();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Hood.periodic() does the work to maintain correct Hood angle.
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
