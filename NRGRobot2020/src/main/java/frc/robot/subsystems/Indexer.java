package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private Victor indexerVictor = new Victor(4);
  /**
   * Creates a new Indexer.
   */
  public Indexer() {

  }
  public void rawIndexer(double power){
    indexerVictor.set(power * 0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
