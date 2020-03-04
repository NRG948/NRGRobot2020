package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;

/**
 * Enables the Turret PID and leaves it running after this command exits.
 */
public class AutoTurret extends CommandBase {
  private final Turret turret;
  private double skewAngle;
  private double maxPower;
  private boolean useDefaultMaxPower = true;

  /**
   * Creates a new AutoTurret.
   */
  public AutoTurret(Turret turret) {
    this.turret = turret;
    this.skewAngle = 0;
    addRequirements(turret);
  }

  /** Optionally sets the maximum power used to rotate the turret. */
  public AutoTurret withMaxPower(double maxPower) {
    this.maxPower = maxPower;
    this.useDefaultMaxPower = false;
    return this;
  }

  /** Optionally sets an angular skew to use while centering the turret on the Limelight target. */
  public AutoTurret withSkew(double skewAngle){
    this.skewAngle = skewAngle;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (useDefaultMaxPower) {
      maxPower = NRGPreferences.TURRET_MOTOR_POWER.getValue();
    }
    double toleranceDegrees = 1.0;
    turret.turretAnglePIDInit(skewAngle, maxPower, toleranceDegrees, true);
    Logger.commandInit(this, String.format("skew:%4.1f toler:%4.1f", skewAngle, toleranceDegrees));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Turret.periodic sends power to the motors using a subsystem PIDController
  }
  
  // Command always exits immediately but leaves PID running.
  @Override
  public boolean isFinished() {
    return true;
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      turret.turretAngleEnd();
    }
    Logger.commandEnd(this);
  }
}
