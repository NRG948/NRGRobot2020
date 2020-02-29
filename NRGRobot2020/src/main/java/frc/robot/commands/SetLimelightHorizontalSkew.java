package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class SetLimelightHorizontalSkew extends CommandBase {
  private Turret turret;
  private double skew;

  /**
   * Creates a new SetLimelightHorizontalSkew.
   */
  public SetLimelightHorizontalSkew(Turret turret, double skew) {
    this.turret = turret;
    this.skew = skew;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.setHorizontalSkew(this.skew);
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
    return true;
  }
}
