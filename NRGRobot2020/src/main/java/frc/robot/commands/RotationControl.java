package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ControlPanelSpinner;
import frc.robot.utilities.Logger;

public class RotationControl extends CommandBase {
  private final ControlPanelSpinner panelSpinner;
  private double maxPower;
  private char previousColor;
  private char currentColor;
  private double rotations = 0;
  /**
   * Creates a new RotationControl.
   */
  public RotationControl(final ControlPanelSpinner panelSpinner) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.panelSpinner = panelSpinner;
  }

  public RotationControl withMaxPower(double maxPower) {
    this.maxPower = maxPower;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.currentColor = panelSpinner.getColor();
    Logger.commandInit(this, "color: " + currentColor);
    this.previousColor = currentColor;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    panelSpinner.spin(maxPower);
    currentColor = panelSpinner.getColor();
    if (currentColor != previousColor && currentColor != 'U') {
      rotations += 0.125;
      previousColor = currentColor;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    panelSpinner.stopMotor();
    Logger.commandEnd(this, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /**
     * We need [3, 5) rotations, but should allow for some extra (3.25) in case of error.
     * We can adjust this number (higher or lower) or create a more accurate algorithm for detecting
     * color changes if the current one doesn't work well.
     */
    return rotations >= 3.25;
  }
}
