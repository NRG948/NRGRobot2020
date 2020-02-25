package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ControlPanelConstants;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Subsystem which commands the Control Panel color spinner.
 */
public class ControlPanelSpinner extends SubsystemBase {
  private Victor panelMotor = new Victor(ControlPanelConstants.kPanelSpinnerMotorPort);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();
  private String color = "Unknown";

  /**
   * These number values currently work for blue, green, red, and yellow,
   * but will probably need to be calibrated at competitions.
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  /**
   * Creates the Control-Panel Spinner subsystem.
   */
  public ControlPanelSpinner() {
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Color detected", color);
  }

  /**
   * Sets the speed of the panel motor.
   * @param speed
   */
  public void spin(double speed) {
    panelMotor.set(speed);
  }

  public void stopMotor() {
    panelMotor.stopMotor();
  }

  /**
   * 
   * @return The color that the color sensor is detecting: Blue, Red, Yellow, Green, or Unknown
   */
  public char getColor() {
    /**
     * From REVRobotics:
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = colorSensor.getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      color = "Blue";
      return 'B';
    } else if (match.color == kRedTarget) {
      color = "Red";
      return 'R';
    } else if (match.color == kGreenTarget) {
      color = "Green";
      return 'G';
    } else if (match.color == kYellowTarget) {
      color = "Yellow";
      return 'Y';
    } else {
      color = "Unknown";
      return 'U';
    }
  }
  public void initShuffleboard(){
    if (!NRGPreferences.SHUFFLEBOARD_CP_SPINNER_ENABLED.getValue()){
      return;
    }
    
    ShuffleboardTab controlPanelSpinnerTab = Shuffleboard.getTab("Control Panel Spinner");

    ShuffleboardLayout controlPanelSpinnerLayout = controlPanelSpinnerTab.getLayout("Control Panel Spinner", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    
  }
}
