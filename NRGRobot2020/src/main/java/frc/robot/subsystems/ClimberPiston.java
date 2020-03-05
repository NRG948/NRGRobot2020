package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberPiston extends SubsystemBase {
    public DoubleSolenoid climberSolenoid = new DoubleSolenoid(4, 5);

    State state = State.RETRACT;
    
    public enum State {
      EXTEND, RETRACT;
    }

    public ClimberPiston() {
    }

    public void toggleState() {
      state = state == State.EXTEND ? State.RETRACT : State.EXTEND;
      climberSolenoid.set(state == State.EXTEND ? Value.kForward : Value.kReverse); 
    }

    public void setState(State state) {
      climberSolenoid.set(state == State.EXTEND ? Value.kForward : Value.kReverse);
      this.state = state;
    }

  @Override
  public void periodic() {
  }
}
