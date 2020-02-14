package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;


public class Indexer extends SubsystemBase {
  private Victor indexerVictor = new Victor(9);
  private double sentPower;
  private SimpleWidget indexerRawOutputWidget;
  /**
   * Creates a new Indexer.
   */
  public Indexer() {

  }
  public void rawIndexer(double power){
    sentPower = power * 0.5;
    indexerRawOutputWidget.getEntry().setDouble(sentPower);
    indexerVictor.set(sentPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void initShuffleboard(){
    ShuffleboardTab indexerTab = Shuffleboard.getTab("Indexer");

    ShuffleboardLayout indexerLayout = indexerTab.getLayout("Indexer", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    indexerRawOutputWidget = indexerLayout.add("Raw Output", 0.0);
  }
}
