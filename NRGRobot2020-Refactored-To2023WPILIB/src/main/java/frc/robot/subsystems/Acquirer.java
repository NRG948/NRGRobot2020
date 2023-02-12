package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Subsystem which controls the vectoring ball intake rollers that extends
 * beyond the bumper.
 * 
 * The pistons that extend/retract the acquirer are in the AcquirerPistons subsystem.
 */
public class Acquirer extends SubsystemBase {
  
  public enum State {
    INTAKING, EJECTING, STATIC;
  }
  
  private static final double MAX_ACQUIRER_POWER = 1;
  
  private final PWMSparkMax acquirerMotor = new PWMSparkMax(8); // used to be 4
  public State state;

  /** Creates the Acquirer subsystem. */
  public Acquirer() {
    acquirerMotor.setInverted(!NRGPreferences.IS_PRACTICE_BOT.getValue());
  }

  public void rawAcquirer(final double power) {
    double sentPower = MathUtil.clamp(power, -MAX_ACQUIRER_POWER, MAX_ACQUIRER_POWER);
    if (sentPower > 0) {
      state = State.INTAKING;
    } else if (sentPower < 0) {
      state = State.EJECTING;
    } else {
      state = State.STATIC;
    }
    acquirerMotor.set(sentPower);
  }

  public void stop() {
    acquirerMotor.stopMotor();
  }

  public State getState() {
    return state;
  }

  @Override
  public void periodic() {
  }

  public void initShuffleboard() {
    if (!NRGPreferences.SHUFFLEBOARD_ACQUIRER_ENABLED.getValue()){
      return;
    }

    final ShuffleboardTab acquirerTab = Shuffleboard.getTab("Acquirer");
    final ShuffleboardLayout acquirerLayout = acquirerTab.getLayout("Acquirer", BuiltInLayouts.kList).withPosition(0, 0)
        .withSize(2, 4);
    acquirerLayout.addNumber("Raw Output", () -> acquirerMotor.get());
  }
}