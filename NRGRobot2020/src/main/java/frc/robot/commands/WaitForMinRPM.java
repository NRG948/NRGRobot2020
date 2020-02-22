
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.NRGPreferences;

public class WaitForMinRPM extends CommandBase {

  ShooterRPM shooterRPM;
  double rpm;
  double rpmOffset;
  /**
   * Creates a new WaitForMinRPM.
   */
  public WaitForMinRPM(double rpm, ShooterRPM shooterRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterRPM);
    this.shooterRPM = shooterRPM;
    this.rpm = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rpmOffset = NRGPreferences.WAIT_FOR_RPM_OFFSET.getValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterRPM.getActualRPM() >= rpm - rpmOffset;
  }
}
