package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Subsystem which commands the ball feeder.
 * 
 * The ball feeder is a single motorized wheel that pushes balls up into contact
 * with the shooter flywheel.
 */
public class Feeder extends SubsystemBase {

  private Victor feederMotor = new Victor(5);

  /**
   * Creates the Feeder subsystem.
   */
  public Feeder() {
    feederMotor.setInverted(true);
  }

  public void rawFeeder(double power){
    feederMotor.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initShuffleboard(){
    if (!NRGPreferences.SHUFFLEBOARD_FEEDER_ENABLED.getValue()){
      return;
    }
    
    ShuffleboardTab feederTab = Shuffleboard.getTab("Feeder");
    ShuffleboardLayout feederLayout = feederTab.getLayout("Feeder", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    feederLayout.addNumber("Raw Output", () -> this.feederMotor.get());
  }
}
