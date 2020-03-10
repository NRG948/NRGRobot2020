package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * The robot subsystem that controls the climber winch motor.
 * 
 * The winch uses a ratcheting mechanism that only allows it to turn
 * in the "climb upward" direction (positive motor power). Running the
 * winch motor in the negative direction could break the climber system.
 */
public class ClimberWinch extends SubsystemBase {
  private final PWMSparkMax climberWinchMotor = new PWMSparkMax(ClimberConstants.kClimberWinchMotorPort);

  /**
   * Creates the ClimberWinch subsystem.
   */
  public ClimberWinch() {
    climberWinchMotor.setInverted(true);
  }
  
  public void rawClimbUp(double power) {
    assert(power >= 0);
    climberWinchMotor.set(power);
  }

  public void stopMotor() {
    climberWinchMotor.stopMotor();
  }

  @Override
  public void periodic() {
  }
}
