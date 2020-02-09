package frc.robot.commands;

import frc.robot.subsystems.Drive;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (NRGPreferences.DRIVE_USE_XBOX_CONTROL.getValue()) {
      m_drive.arcadeDrive(-m_xboxController.getY(Hand.kLeft), m_xboxController.getX(Hand.kRight));
    } else {
      m_drive.tankDrive(-m_leftJoystick.getY(), -m_rightJoystick.getY(), true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
