package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;

public class ManualHood extends CommandBase {

  private final Hood hood;
  private final XboxController m_xboxController;
  private double power = 0;

  /**
   * Creates a new ManualHood.
   */
  public ManualHood(Hood hood, XboxController xboxController) {
    this.hood = hood;
    m_xboxController = xboxController;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_xboxController.getPOV() == 0) {
      power = NRGPreferences.HOOD_MANUAL_MOTOR_POWER.getValue();
    } else if (m_xboxController.getPOV() == 180) {
      power = -NRGPreferences.HOOD_MANUAL_MOTOR_POWER.getValue();
    } else {
      power = 0;
    }
    hood.rawHood(power);
  }
  
  // Manual commands never end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.hoodEnd();
    Logger.commandEnd(this);
  }
}
