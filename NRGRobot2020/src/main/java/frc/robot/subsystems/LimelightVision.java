package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightVision extends SubsystemBase {
  public static final String kDefaultAuto = "Default";
  public static final String kCustomAuto = "My Auto";
  public String m_autoSelected;
  public final SendableChooser<String> m_chooser = new SendableChooser<>();
  public final double visionAngle = 22; // limelight mounting angle
  public final double h2 = 83.75; // height of high target
  public final double h1 = 38; // mounting height
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
    return (h2 - h1) / Math.tan(Math.toRadians(visionAngle + ty.getDouble(0)));
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

  public double getTv(){
    return tv.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // post to smart dashboard periodically
  }
}
