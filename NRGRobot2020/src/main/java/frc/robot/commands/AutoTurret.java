package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

/**
 * Enables Turret PID and leaves it running after command exits.
 */
public class AutoTurret extends CommandBase {
  private final Turret turret;
  private double maxPower;

  /**
   * Creates a new AutoTurret.
   */
  public AutoTurret(final Turret turret) {
    this.turret = turret;

    addRequirements(turret);
  }

  public AutoTurret withMaxPower(double maxPower) {
    this.maxPower = maxPower;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.turretAnglePIDInit(0, maxPower, 2.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Turret.periodic() takes care of updating controller
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.turretAngleEnd();
  }

  // Command always exits immediately but leaves PID running.
  @Override
  public boolean isFinished() {
    return true;
  }
}
