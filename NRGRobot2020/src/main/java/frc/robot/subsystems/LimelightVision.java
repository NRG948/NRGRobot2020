/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.utilities.*;

public class LimelightVision extends SubsystemBase {
  public static final String kDefaultAuto = "Default";
  public static final String kCustomAuto = "My Auto";
  public String m_autoSelected;
  public final SendableChooser<String> m_chooser = new SendableChooser<>();
  public final double visionAngle = 0; // limelight mounting angle
  public final double h2 = 83.75; // height of high target
  public final double h1 = 38; // mounting height
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");
  // read values periodically
  boolean averageDistancebool = false;
  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double skew = ts.getDouble(0.0);
  double z = tshort.getDouble(0.0);
  double tLong = tlong.getDouble(0.0);
  double distanceUsingTshort = (4822 / z) - 5.0664;
  double distanceTLong = (-0.024 + Math.sqrt(-0.0148 * tLong + 1.2543432)) / 0.0074;
  double AngledDistance = Math.sqrt(Math.pow(distanceUsingTshort, 2) + Math.pow(distanceTLong, 2));
  double distance = (h2 - h1) / Math.tan(Math.toRadians(visionAngle + y));
  double average = (distanceUsingTshort + distance) / 2;
  double distanceMax = 0;

  /**
   * Creates a new LimelightVision.
   */
  public LimelightVision() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimeLIghtSkew", skew);
    SmartDashboard.putNumber("Tshort", z);
    SmartDashboard.putNumber("DistanceUsingTshort", distanceUsingTshort);
    SmartDashboard.putNumber("AverageDistance", average);
    SmartDashboard.putNumber("LimeLightDistance", distance);
    SmartDashboard.putNumber("Tlong", distanceTLong);
    SmartDashboard.putNumber("AngledDistance", AngledDistance);
    // post to smart dashboard periodically
  }
}
