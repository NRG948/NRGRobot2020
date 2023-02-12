package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;

public class WaitForMinRPM extends CommandBase {

  private final ShooterRPM shooterRPM;

  private double rpm;
  private double rpmOffset;

  /** Creates a new WaitForMinRPM command. */
  public WaitForMinRPM(double rpm, ShooterRPM shooterRPM) {
    this.shooterRPM = shooterRPM;
    this.rpm = rpm;
    addRequirements(shooterRPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rpmOffset = NRGPreferences.WAIT_FOR_RPM_OFFSET.getValue();
    Logger.commandInit(this, "offset:" + rpmOffset);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterRPM.getActualRPM() >= rpm - rpmOffset;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this, interrupted);
  }
}
