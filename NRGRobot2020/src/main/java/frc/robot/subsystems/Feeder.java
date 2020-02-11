package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem which commands the ball feeder.
 * 
 * The ball feeder is a single motorized wheel that pushes balls up into contact
 * with the shooter flywheel.
 */
public class Feeder extends SubsystemBase {
  private Victor feederMotor = new Victor(5);

  /**
   * Creates the Feeder subsystem.
   */
  public Feeder() {
  }

  public void rawFeeder(double power){
    feederMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
