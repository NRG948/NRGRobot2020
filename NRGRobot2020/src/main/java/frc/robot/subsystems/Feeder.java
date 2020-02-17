package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Subsystem which commands the ball feeder.
 * 
 * The ball feeder is a single motorized wheel that pushes balls up into contact
 * with the shooter flywheel.
 */
public class Feeder extends SubsystemBase {

  private DigitalInput beamBreak = new DigitalInput(8);
  private Victor feederMotor = new Victor(5);
  private SimpleWidget feederRawOutputWidget;  

  /**
   * Creates the Feeder subsystem.
   */
  public Feeder() {
  }

  public void rawFeeder(double power){
    feederRawOutputWidget.getEntry().setDouble(power);
    feederMotor.set(power);
  }

  public boolean isBallInShootingPosition(){
    return beamBreak.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initShuffleboard(){
    ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");
    ShuffleboardLayout feederLayout = feederTab.getLayout("Feeder", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    feederRawOutputWidget = feederLayout.add("Raw Output", 0.0);
  }
}
