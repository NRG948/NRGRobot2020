package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;

/**
 * Maintains a given RPM speed on the shooter wheel using a Take-Back-Half feedback loop.
 */
public class MaintainShooterRPM extends CommandBase {

  private final ShooterRPM shooterRPM;

  private double goalRPM;
  private boolean setAndExit;
  private boolean useDefaultGoalRPM = true;

  /** Creates a new SetShooterRPM. */
  public MaintainShooterRPM(ShooterRPM shooterRPM) {
    this.goalRPM = Double.NaN;
    this.shooterRPM = shooterRPM;
    addRequirements(shooterRPM);
  }

  public MaintainShooterRPM atRpm(double goalRPM) {
    this.useDefaultGoalRPM = false;
    this.goalRPM = goalRPM;
    return this;
  }

  public MaintainShooterRPM setAndExit() {
    this.setAndExit = true;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (useDefaultGoalRPM) {
      goalRPM = NRGPreferences.SHOOTER_TEST_RPM.getValue();
    }
    shooterRPM.setGoalRPM(goalRPM);
    Logger.commandInit(this, String.format("rpm:%4.0f", goalRPM));
  }

  @Override
  public void execute() {
    // ShooterRPM.periodic() runs the take-back-half feedback control.
  }

  // This command either ends immediately (if in set-and-exit mode), or runs forever.
  @Override
  public boolean isFinished() {
    return setAndExit;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!setAndExit) {
      shooterRPM.disableTakeBackHalf();
    }
    Logger.commandEnd(this);
  }
}
