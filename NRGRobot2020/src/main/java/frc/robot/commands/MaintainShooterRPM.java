package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.NRGPreferences;

public class MaintainShooterRPM extends CommandBase {
  private final ShooterRPM shooterRPM;

  private double goalRPM;
  private Timer timer = new Timer();
  private double seconds = 20;
  private boolean setAndExit;

  /**
   * Creates a new SetShooterRPM.
   */
  public MaintainShooterRPM(ShooterRPM shooterRPM) {
    this.goalRPM = Double.NaN;
    this.shooterRPM = shooterRPM;
    addRequirements(shooterRPM);
  }

  public MaintainShooterRPM atRpm(double goalRPM) {
    this.goalRPM = goalRPM;
    return this;
  }

  public MaintainShooterRPM forSeconds(double seconds) {
    this.seconds = seconds;
    return this;
  }

  public MaintainShooterRPM setAndExit() {
    this.setAndExit = true;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if (Double.isNaN(goalRPM)) {
      goalRPM = NRGPreferences.SHOOTER_TEST_RPM.getValue();
    }
    shooterRPM.setGoalRPM(goalRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (setAndExit || timer.get() > seconds) {
      timer.stop();
      return true;
    }
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!setAndExit) {
      shooterRPM.disableTakeBackHalf();
    }
  }
}
