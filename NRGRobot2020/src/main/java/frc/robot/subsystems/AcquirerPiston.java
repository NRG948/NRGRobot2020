package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.NRGPreferences;

public class AcquirerPiston extends SubsystemBase {
    public DoubleSolenoid acquirerSolenoid;

    State state;
    
    public enum State {
      EXTEND, RETRACT;
    }

    public AcquirerPiston() {
      if (NRGPreferences.USING_PRACTICE_BOT.getValue()) {
        acquirerSolenoid = new DoubleSolenoid(2, 3);
      } else {
        acquirerSolenoid = new DoubleSolenoid(3, 2);
      }
      this.state = acquirerSolenoid.get() == Value.kForward ? State.EXTEND : State.RETRACT;
    }

    public void toggleState() {
      state = state == State.EXTEND ? State.RETRACT : State.EXTEND;
      acquirerSolenoid.set(state == State.EXTEND ? Value.kForward : Value.kReverse); 
    }

    public void setState(State state) {
      acquirerSolenoid.set(state == State.EXTEND ? Value.kForward : Value.kReverse);
      this.state = state;
    }

  @Override
  public void periodic() {
  }
}
