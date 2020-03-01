package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquirer;
import frc.robot.utilities.Logger;

public class ManualAcquirer extends CommandBase {
  Acquirer acquirer;
  final XboxController m_xboxController;

  /** Creates a new ManualAcquirer. */
  public ManualAcquirer(Acquirer acquirer, final XboxController xboxController) {
    this.acquirer = acquirer;
    this.m_xboxController = xboxController;
    addRequirements(acquirer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    acquirer.rawAcquirer(m_xboxController.getY(Hand.kRight));
  }
  
  // Manual commands never end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this);
  }
}
