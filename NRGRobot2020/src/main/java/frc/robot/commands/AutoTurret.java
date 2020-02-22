package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.NRGPreferences;

/**
 * Enables Turret PID and leaves it running after command exits.
 */
public class AutoTurret extends CommandBase {
  private final Turret turret;
  private final LimelightVision limelightVision;
  private double maxPower;
  private boolean useDefaultMaxPower = true;
  private int noTargetCount = 0;

  /**
   * Creates a new AutoTurret.
   */
  public AutoTurret(Turret turret, LimelightVision limelightVision) {
    this.turret = turret;
    this.limelightVision = limelightVision;
    addRequirements(turret);
  }

  public AutoTurret withMaxPower(double maxPower) {
    this.maxPower = maxPower;
    this.useDefaultMaxPower = false;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (useDefaultMaxPower) {
      maxPower = NRGPreferences.TURRET_MOTOR_POWER.getValue();
    }
    turret.turretAnglePIDInit(0, maxPower, 1, true);
    noTargetCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Turret.periodic sends power to the motors using a subsystem PIDController
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    boolean hasTarget = limelightVision.getTv()==1;
    if(interrupted || (!hasTarget && noTargetCount++ >= 5)){
      turret.turretAngleEnd();
    } else if(hasTarget){
      noTargetCount = 0;
    }
  }

  // Command always exits immediately but leaves PID running.
  @Override
  public boolean isFinished() {
    return true;
    // return turret.turretAngleOnTarget();
  }
}
