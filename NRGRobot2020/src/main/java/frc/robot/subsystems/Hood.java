package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
  private Victor hoodVictor = new Victor(5);
  /**
   * Creates a new Hood.
   */
  public Hood() {

  }
  //TODO: Add encoders for hard limit
  public void rawHood(double power){
    hoodVictor.set(power * 0.5);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
