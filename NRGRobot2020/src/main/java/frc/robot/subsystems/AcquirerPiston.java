package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AcquirerPiston extends SubsystemBase {
    public DoubleSolenoid acquirerSolenoid = new DoubleSolenoid(2, 3);

    State state = State.RETRACT;
    
    public enum State {
      EXTEND, RETRACT;
    }

    public AcquirerPiston() {
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
