package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberWinch extends SubsystemBase {
  private final PWMSparkMax climberWinchMotor = new PWMSparkMax(ClimberConstants.kClimberWinchMotorPort);
  /**
   * Creates a new ClimberWinch.
   */
  public ClimberWinch() {
    climberWinchMotor.setInverted(true);
  }
  
  public void rawClimb(double power) {
    climberWinchMotor.set(power);
  }

  public void stopMotor() {
    climberWinchMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }
}
