package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.Logger;

public class ManualFeeder extends CommandBase {
  Feeder feeder;
  final XboxController m_xboxController;  /**
   * Creates a new ManualFeeder.
   */
  public ManualFeeder(Feeder feeder, final XboxController xboxController) {
    this.feeder = feeder;
    this.m_xboxController = xboxController;
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.rawFeeder(m_xboxController.getY(Hand.kLeft) * 0.5);
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
