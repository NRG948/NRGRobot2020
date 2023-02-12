package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem to control the piston that extends/retracts the climber 4-bar mechanism.
 */
public class ClimberPiston extends SubsystemBase {
// quick fixed DoubleSolenoid, a
  public final DoubleSolenoid climberSolenoid = new DoubleSolenoid(null, 5, 4);

  /** Define the possible climber states. */
  public enum State {
    EXTEND, RETRACT;
  }

  /** Creates the ClimberPiston subsystem. */
  public ClimberPiston() {
  }

  /** Returns the current state of the climber piston. */
  public State getState() {
    return climberSolenoid.get() == Value.kForward ? State.EXTEND : State.RETRACT;
  }

  /** Sets a new state for the climber piston. */
  public void setState(State state) {
    climberSolenoid.set(state == State.EXTEND ? Value.kForward : Value.kReverse);
  }

  /** Toggles the state of the climber piston. */
  public void toggleState() {
    setState(getState() == State.EXTEND ? State.RETRACT : State.EXTEND);
  }

  @Override
  public void periodic() {
  }
}
