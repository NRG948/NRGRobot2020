package frc.robot.commands;

import frc.robot.subsystems.Drive;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * An example command that uses an example subsystem.
 */
public class ManualDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Drive m_drive;
  private final Joystick m_rightJoystick;
  private final Joystick m_leftJoystick;
  private final XboxController m_xboxController;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ManualDrive(final Drive subsystem, final Joystick leftJoystick, final Joystick rightJoystick, final XboxController xboxController) {
    m_drive = subsystem;
    m_leftJoystick = leftJoystick;
    m_rightJoystick = rightJoystick;
    m_xboxController = xboxController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isAcquirerFront = m_drive.getIsAcquirerFront();
    // NRGPreferences.DRIVE_ACQUIRER_IS_FRONT.getValue();

    if (NRGPreferences.DRIVE_USE_XBOX_CONTROL.getValue()) {
      double y = m_xboxController.getLeftY() * (isAcquirerFront ? -1.0 : 1.0);
      m_drive.arcadeDrive(y, m_xboxController.getRightX());
    } else {
      if (isAcquirerFront) {
        m_drive.tankDrive(-m_leftJoystick.getY(), -m_rightJoystick.getY(), true);
      } else {
        m_drive.tankDrive(m_rightJoystick.getY(), m_leftJoystick.getY(), true);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    Logger.commandEnd(this, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
