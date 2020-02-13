package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Subsystem which controls the vectoring ball intake rollers that extends beyond the bumper.
 * 
 * The pistons that extend/retract the acquirer are a separate subsystem.
 */
public class Acquirer extends SubsystemBase {

  private double sentPower;
  private SimpleWidget acquirerRawOutputWidget;

  private static final double MAX_ACQUIRER_POWER = 0.5;

  private Victor acquirerMotor = new Victor(4);

  /** Creates a new Acquirer. */
  public Acquirer() {
  }

  public void rawAcquirer(double power){
    sentPower = MathUtil.clamp(power, -MAX_ACQUIRER_POWER, MAX_ACQUIRER_POWER);
    acquirerRawOutputWidget.getEntry().setDouble(sentPower);
    acquirerMotor.set(sentPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initShuffleboard(){
    ShuffleboardTab acquirerTab = Shuffleboard.getTab("Acquirer");

    ShuffleboardLayout acquirerLayout = acquirerTab.getLayout("Acquirer", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    acquirerRawOutputWidget = acquirerLayout.add("Raw Output", 0.0);
  }
}