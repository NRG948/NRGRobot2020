package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A Subsystem to control gearbox shifting.
 */
public class Gearbox extends SubsystemBase {

  public enum Gear {
    HIGH, LOW;
  }
// quick fixed this code, i dont know what it did - Jamal Jackson
  private DoubleSolenoid gearboxSolenoid = new DoubleSolenoid(null, 0, 1); 
  private Gear state = Gear.HIGH;

  public void setHighGear() {
    gearboxSolenoid.set(Value.kForward);
    state = Gear.HIGH;
  }

  public void setLowGear() {
    gearboxSolenoid.set(Value.kReverse);
    state = Gear.LOW;
  }

  public void toggleGears() {
    if (state == Gear.HIGH) {
      setLowGear();
    } else {
      setHighGear();
    }
  }

  public Gear getState() {
    return this.state;
  }
}