package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Logger;

public class StopTurretAnglePID extends CommandBase {
  private Turret turret;
  
  /**
   * Creates a new StopAutoFeeder command.
   */
  public StopTurretAnglePID(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this);
    turret.turretAngleEnd();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this, interrupted);
  }

  // Returns true when the command should end, which is immediately.
  @Override
  public boolean isFinished() {
    return true;
  }
}
