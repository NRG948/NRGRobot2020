package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Subsystem which controls the vectoring ball intake rollers that extends
 * beyond the bumper.
 * 
 * The pistons that extend/retract the acquirer are a separate subsystem.
 */
public class Acquirer extends SubsystemBase {

  private double sentPower;
  


  private static final double MAX_ACQUIRER_POWER = 1;

  private final PWMSparkMax acquirerMotor = new PWMSparkMax(4);

  /** Creates a new Acquirer. */
  public Acquirer() {
    acquirerMotor.setInverted(!NRGPreferences.USING_PRACTICE_BOT.getValue());
  }

  public void rawAcquirer(final double power) {
    sentPower = MathUtil.clamp(power, -MAX_ACQUIRER_POWER, MAX_ACQUIRER_POWER);
    acquirerMotor.set(sentPower);
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

  public void stop() {
    acquirerMotor.stopMotor();
  }
}