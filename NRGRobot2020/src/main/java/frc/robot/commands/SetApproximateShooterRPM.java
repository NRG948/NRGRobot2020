
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.Logger;

public class SetApproximateShooterRPM extends CommandBase {
  ShooterRPM shooterRPM;
  double rpm;
  /**
   * Creates a new SetShooterApproximateRPM.
   */
  public SetApproximateShooterRPM(double rpm, ShooterRPM shooterRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterRPM);
    this.rpm = rpm;
    this.shooterRPM = shooterRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this, String.format("%.0f", rpm));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterRPM.setFlyWheel(shooterRPM.guessMotorOutputForRPM(rpm));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
