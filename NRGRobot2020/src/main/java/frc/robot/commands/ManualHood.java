package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class ManualHood extends CommandBase {
  Hood hood;
  private double power = 0;
  private final XboxController m_xboxController;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_xboxController.getPOV() == 0){
      power = 0.2;
    }
    else if(m_xboxController.getPOV() == 180){
      power = -0.2;
    }
    else{
      power = 0;
    }
    hood.rawHood(power);
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.hoodEnd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
