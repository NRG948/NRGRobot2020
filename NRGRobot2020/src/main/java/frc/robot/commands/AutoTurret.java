package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;

/**
 * Enables Turret PID and leaves it running after command exits.
 */
public class AutoTurret extends CommandBase {
  private final Turret turret;
  private final LimelightVision limelightVision;
  private double skewAngle;
  private double maxPower;
  private boolean useDefaultMaxPower = true;

  /**
   * Creates a new AutoTurret.
   */
  public AutoTurret(Turret turret, LimelightVision limelightVision) {
    this.turret = turret;
    this.limelightVision = limelightVision;
    this.skewAngle = 0;
    addRequirements(turret);
  }

  public AutoTurret withMaxPower(double maxPower) {
    this.maxPower = maxPower;
    this.useDefaultMaxPower = false;
    return this;
  }

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
    turret.turretAnglePIDInit(skewAngle, maxPower, 1, true);
    Logger.commandInit(this, String.format("skew:%4.1f toler:%4.1f", skewAngle, 1.0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Turret.periodic sends power to the motors using a subsystem PIDController
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
