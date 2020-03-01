package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Hood extends SubsystemBase {

  private static final double HOOD_BACK_VOLTAGE_PRACTICE = 3.99;
  private static final double HOOD_FORWARD_VOLTAGE_PRACTICE = 1.5;
  private static final int MAX_LIMIT = 100;
  private static final int LOWER_HARD_STOP = 1;
  private static final int UPPER_HARD_STOP = 95;
  private Victor hoodMotor = new Victor(TurretConstants.kHoodMotorPort);
  private AnalogInput encoderInput = new AnalogInput(1);
  private AnalogEncoder hoodEncoder = new AnalogEncoder(encoderInput);

  /**
   * Creates a new Hood.
   */
  public Hood() {
    // TODO Find distance per revolution for hood so that range of values is 0 - 100.
    // hoodEncoder.setDistancePerRotation(TurretConstants.kHoodDistancePerRevolution);
  }

  public void reset() {
    //hoodEncoder.reset();
  }

  /**
   * Turns motor with raw input. Has hard-stop protections.
   * @param power motor power, + means forward, - means backwards
   */
  public void rawHood(double power) {
    power = MathUtil.clamp(power, -0.5, 0.5);
    double hoodPosition = getPosition();
    // Prevent the turret from turning past hard stops
    if (hoodPosition >= UPPER_HARD_STOP && power > 0 || hoodPosition < LOWER_HARD_STOP && power < 0) {
      power = 0;
    }
    hoodMotor.set(power);
  }

  /** Returns the position of the hood, scaled to be between 0 (full back) and 100 (full forward). */
  public double getPosition() {
    return MAX_LIMIT * (HOOD_BACK_VOLTAGE_PRACTICE - hoodEncoder.get()) 
          / (HOOD_BACK_VOLTAGE_PRACTICE - HOOD_FORWARD_VOLTAGE_PRACTICE);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initShuffleboard() {
    if (!NRGPreferences.SHUFFLEBOARD_HOOD_ENABLED.getValue()){
      return;
    }
    
    ShuffleboardTab hoodTab = Shuffleboard.getTab("Hood");

    ShuffleboardLayout hoodLayout = hoodTab.getLayout("Hood", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    hoodLayout.addNumber("Raw Output", () -> this.hoodMotor.get());
    hoodLayout.addNumber("Position", () -> this.getPosition());
    hoodLayout.add("Encoder", this.hoodEncoder);
    hoodLayout.add("Analog Input", this.encoderInput);
  }

  public void hoodEnd() {
    this.hoodMotor.set(0);
  }
}
