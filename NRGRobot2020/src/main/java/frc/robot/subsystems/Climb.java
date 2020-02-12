package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Climb extends SubsystemBase {

  /**
   * Creates the climber subsystem.
   */
  public Climb() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void initShuffleboard(){
    ShuffleboardTab climbTab = Shuffleboard.getTab("Climb");

    ShuffleboardLayout climbLayout = climbTab.getLayout("Climb", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
  }
}
