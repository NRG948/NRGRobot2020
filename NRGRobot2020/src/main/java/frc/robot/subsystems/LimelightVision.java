package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.NRGPreferences;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightVision extends SubsystemBase {
  public final double TARGET_HEIGHT = 83.75; // height of high target in inches
  public final double LIMELIGHT_MOUNTING_HEIGHT = 24.0; // mounting height in inches
  public final double LIMELIGHT_MOUNTING_ANGLE = Math.toRadians(22); // limelight mounting angle
  public final double LIMELIGHT_CENTER_Y = 240/2; 
  public final double LIMELIGHT_HALF_FOV_Y = Math.toRadians(49.7/2);
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");
  NetworkTableEntry thor = table.getEntry("thor");
  NetworkTableEntry tvert = table.getEntry("tver");
  // read values periodically

  double distanceUsingTshort = (4822 / tshort.getDouble(0.0)) - 5.0664; //Keeping this because it was hard to figure out


  private boolean ledToggle = false;
  /**
   * Creates a new LimelightVision.
   */
  public LimelightVision() {
    table.getEntry("ledMode").setNumber(1); // sets defualt led mode to off;

  }
  // Turns off Led
  public void turnOffLed(){
    table.getEntry("ledMode").setNumber(1);
  }
  
  // Turns on Led
  public void turnOnLed(){
    table.getEntry("ledMode").setNumber(3);
  }

  public void toggleLed(){
    ledToggle = !ledToggle;
    table.getEntry("ledMode").setNumber(ledToggle ? 3 : 1);
  }
  /**
   * Returns the horizontal angle to the target.
   * 
   * @return The horizontal angle to the target with positive being counterclockwise
   */
  public double getX() {
    return tx.getDouble(0); 
  }

  public double getDistance() {
    if(!hasTarget()) {
      return 0.0;
    }
    // return 4822.0 / tshort.getDouble(0.0) - 5.0664; // Formula 1
    // return (TARGET_HEIGHT - LIMELIGHT_MOUNTING_HEIGHT) / Math.tan(LIMELIGHT_MOUNTING_ANGLE + Math.toRadians(getAngle())); // Formula 2
    return (LIMELIGHT_CENTER_Y / Math.tan(LIMELIGHT_HALF_FOV_Y) / Math.cos(Math.toRadians(getAngle()))) * (17.0 / tshort.getDouble(0.000001)); // Formula 3
  }

  public double getAngle() {
    return ty.getDouble(0.0);
  }

  public double getWidth() {
    return thor.getDouble(0.0);
  }

  public double getHeight() {
    return tvert.getDouble(0.0);
  }

  public double getSkew() {
    return ts.getDouble(0.0);
  }

  /**
   * Returns true if the Limelight has target.
   */
  public boolean hasTarget() {
    return tv.getDouble(0.0) != 0.0;
  }

  @Override
  public void periodic() {
    // post to smart dashboard periodically
  }

  public void addShuffleboardTab() {
    if (!NRGPreferences.SHUFFLEBOARD_LIMELIGHT_ENABLE.getValue()) {
      return;
    }

    ShuffleboardTab limelightTab = Shuffleboard.getTab("Limelight");
    ShuffleboardLayout limelightLayout = limelightTab.getLayout("Limelight", BuiltInLayouts.kList);
    limelightLayout.addNumber("tshort", () -> tshort.getDouble(0.0));
    limelightLayout.addNumber("ty", () -> ty.getDouble(0.0));
    limelightLayout.addNumber("Distance", this::getDistance);
    limelightLayout.addNumber("Angle", this::getAngle);
    limelightLayout.addNumber("Distance (formula 1)", () -> 4822.0 / tshort.getDouble(0.0) - 5.0664);
    limelightLayout.addNumber("Distance (formula 2)", () -> (TARGET_HEIGHT - LIMELIGHT_MOUNTING_HEIGHT) / Math.tan(LIMELIGHT_MOUNTING_ANGLE + Math.toRadians(getAngle())));
    limelightLayout.addNumber("Distance (formula 3)", () -> (LIMELIGHT_CENTER_Y / Math.tan(LIMELIGHT_HALF_FOV_Y) / Math.cos(Math.toRadians(getAngle()))) * (17.0 / tshort.getDouble(0.000001)));
  }
}
