package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AcquirerPiston extends SubsystemBase {
    public DoubleSolenoid acquirerSolenoid = new DoubleSolenoid(0, 1);
    public AcquirerPiston() {
    }
    
    public enum State {
      EXTEND, RETRACT;
    }

    State state = State.RETRACT;

    public void setState() {
      Value direction = state == State.EXTEND ? Value.kReverse : Value.kForward;
      state = state == State.EXTEND ? State.RETRACT : State.EXTEND;
      acquirerSolenoid.set(direction); 
    }

  @Override
  public void periodic() {
  }
}
