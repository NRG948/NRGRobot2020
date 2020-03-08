package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;

public class ManualTurret extends CommandBase {
  /**
   * Creates a new ManualTurret.
   */
  final XboxController xboxController;
  final Turret turret;
  private double power;

  ; 

  public ManualTurret(Turret turret, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xboxController = xboxController;
    this.turret = turret;
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this);
    turret.turretAngleEnd();
    power = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(xboxController.getPOV() == 90){
      power = -NRGPreferences.TURRET_MOTOR_POWER.getValue();
    }
    else if(xboxController.getPOV() == 270){
      power = NRGPreferences.TURRET_MOTOR_POWER.getValue();
    } else {
      power = 0;
    }
    turret.rawTurret(power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
