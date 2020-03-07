package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.NRGPreferences;

/**
 * Subsystem which controls the two pistons that extend the acquirer rollers
 * beyond the bumper, or retract the rollers to be within the frame perimeter.
 * 
 * A single double-solenoid module controls both pistons.
 * 
 * The motor for the acquirer rollers is controlled by the Acquirer subsystem.
 */
public class AcquirerPistons extends SubsystemBase {

  public final DoubleSolenoid acquirerSolenoid;

  public enum State {
    EXTEND, RETRACT;
  }

  /** Creates the AcquirerPistons subsystem. */
  public AcquirerPistons() {
    if (NRGPreferences.USING_PRACTICE_BOT.getValue()) {
      acquirerSolenoid = new DoubleSolenoid(2, 3);
    } else {
      acquirerSolenoid = new DoubleSolenoid(3, 2);
    }
  }

  /** Returns the current state of the acquirer pistons */
  public State getState() {
    return acquirerSolenoid.get() == Value.kForward ? State.EXTEND : State.RETRACT;
  }

  public void setState(State state) {
    acquirerSolenoid.set(state == State.EXTEND ? Value.kForward : Value.kReverse);
  }

  public void toggleState() {
    setState(getState() == State.EXTEND ? State.RETRACT : State.EXTEND);
  }

  @Override
  public void periodic() {
  }
}
