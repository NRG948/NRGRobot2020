package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;


public class Hood extends SubsystemBase {
  private Victor hoodMotor = new Victor(TurretConstants.kHoodMotorPort);
  private AnalogEncoder hoodEncoder;
  private double sentPower;

  private SimpleWidget rawHoodOutputWidget;
  /**
   * Creates a new Hood.
   */
  public Hood() {

  }
  //TODO: Add encoders for hard limit
  public void rawHood(double power){
    sentPower = power * 0.5;
    rawHoodOutputWidget.getEntry().setDouble(sentPower);
    hoodMotor.set(sentPower);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void initShuffleboard(){
    ShuffleboardTab hoodTab = Shuffleboard.getTab("Hood");
  
    ShuffleboardLayout hoodLayout = hoodTab.getLayout("Hood", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    rawHoodOutputWidget = hoodLayout.add("Raw Output", 0.0);
  }
  public void hoodEnd() {
    this.hoodMotor.set(0);
  }
}
