package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;

public class DisableShooterRPM extends CommandBase {

  ShooterRPM shooterRPM;
  /**
   * Creates a new DisableShooterRPM.
   */
  public DisableShooterRPM(ShooterRPM shooterRPM) {
    this.shooterRPM = shooterRPM;
    addRequirements(shooterRPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterRPM.disableTakeBackHalf();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DisableShooterRPM end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
