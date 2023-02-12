package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Subsystem that controls the fuel cell shooter hood, which controls the vertical angle of shots.
 */
public class Hood extends SubsystemBase {

  /**
   *
   */
  private static final double HOOD_BACK_VOLTAGE_PRACTICE = 4.12;
  private static final double HOOD_VOLTAGE_RANGE_PRACTICE = 2.49;

  private static final double HOOD_BACK_VOLTAGE_COMPETITION = 3.99;  // TODO: measure this!
  private static final double HOOD_VOLTAGE_RANGE_COMPETITION = 2.49;

  private static final int MAX_LIMIT = 100;
  private static final int LOWER_HARD_STOP = 10;
  private static final int UPPER_HARD_STOP = 97;

  private static final double INITIATION_SHOOT_DISTANCE = 160; // inches
  private static final double TRENCH_FAR_SHOOT_DISTANCE = 330; // inches
  private static final double TOLERANCE = 0.1;

  private final Victor hoodMotor = new Victor(TurretConstants.kHoodMotorPort);
  private final AnalogInput encoderInput = new AnalogInput(1);
  private final AnalogEncoder hoodEncoder = new AnalogEncoder(encoderInput);
  private final LimelightVision limelightVision;
  private boolean isAutoHoodEnabled = false;

  /** Creates a new Hood. */
  public Hood(LimelightVision limelightVision) {
    this.limelightVision = limelightVision;
  }

  public void reset() {
    // Do NOT reset the absolute encoder for the hood!
  }

  /**
   * Turns motor with raw input. Has hard-stop protections.
   * @param power motor power, + means forward/up, - means backwards/down.
   */
  public void rawHood(double power) {
    power = MathUtil.clamp(power, -0.5, 0.5);
    double hoodPosition = getPosition();
    // Prevent the turret from moving past hard stops
    if (hoodPosition >= UPPER_HARD_STOP && power > 0 || hoodPosition < LOWER_HARD_STOP && power < 0) {
      power = 0;
    }
    hoodMotor.set(power);
  }

  public void enableAutoHood(){
    isAutoHoodEnabled = true;
  }

  /** Turns off the hood motor. */
  public void hoodEnd() {
    this.hoodMotor.set(0);
    isAutoHoodEnabled = false;
  }

  /** Returns the position of the hood, scaled to be between 0 (full back) and 100 (full forward). */
  public double getPosition() {
    double volts = hoodEncoder.get();
    if (NRGPreferences.IS_PRACTICE_BOT.getValue()) {
      return MAX_LIMIT * (HOOD_BACK_VOLTAGE_PRACTICE - volts) / (HOOD_VOLTAGE_RANGE_PRACTICE);
    } else {
      return MAX_LIMIT * (HOOD_BACK_VOLTAGE_COMPETITION - volts) / (HOOD_VOLTAGE_RANGE_COMPETITION);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(isAutoHoodEnabled){
      if(limelightVision.hasTarget()){
        double goalPosition = hoodPositionFromLimelight(limelightVision.getDistance());
        double currentPosition = getPosition();
        double error = goalPosition - currentPosition; // This is how far off we are from the goal position;
        if(Math.abs(error) < TOLERANCE){
          rawHood(0);
        } else{
          rawHood(NRGPreferences.HOOD_MANUAL_MOTOR_POWER.getValue() * Math.signum(error));
        }
      } else {
        rawHood(0);
      }
    }
  }

  // Interpalates a hood position based on known values of Limelight distance;
  private double hoodPositionFromLimelight(double distance){
    double posTrenchFar = NRGPreferences.HOOD_POSITION_TRENCH_FAR.getValue();
    double posInitiation = NRGPreferences.HOOD_POSITION_INITIATION.getValue();
    double slope = (posTrenchFar - posInitiation) / (TRENCH_FAR_SHOOT_DISTANCE - INITIATION_SHOOT_DISTANCE);
    return slope * distance + posTrenchFar - slope * TRENCH_FAR_SHOOT_DISTANCE;
  }

  public void initShuffleboard() {
    if (!NRGPreferences.SHUFFLEBOARD_HOOD_ENABLED.getValue()) {
      return;
    }

    ShuffleboardTab hoodTab = Shuffleboard.getTab("Hood");
    ShuffleboardLayout hoodLayout = hoodTab.getLayout("Hood", BuiltInLayouts.kList)
      .withPosition(0, 0)
      .withSize(2, 4);

    hoodLayout.addNumber("Raw Output", () -> this.hoodMotor.get());
    hoodLayout.addNumber("Position", () -> this.getPosition());
    hoodLayout.addNumber("Encoder Value", () -> this.hoodEncoder.get());
    hoodLayout.addNumber("Analog Input", () -> this.encoderInput.getVoltage());
  }
}
