package frc.robot.commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;

public class ManualShooter extends CommandBase {
  ShooterRPM shooterRPM;
  XboxController xboxController;
  /**
   * Creates a new ManualShooter.
   */
  public ManualShooter(ShooterRPM shooterRPM, XboxController xboxController) {
    this.shooterRPM = shooterRPM;
    this.xboxController = xboxController;
    addRequirements(shooterRPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterRPM.setFlyWheel(xboxController.getTriggerAxis(Hand.kRight));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
