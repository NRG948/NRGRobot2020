/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.subsystems.RaspberryPiVision;

/**
 * Add your docs here.
 */
public class LoadingStationTarget implements VisionTarget{

    private double distance;
    private double skew;
    private double angle;

    public LoadingStationTarget(double distance, double angle, double skew) {
        this.distance = distance;
        this.angle = angle;
        this.skew = skew;
    }

    @Override
    public double getDistanceInInches() {
        return distance;
    }
    
    @Override
    public double getAngleInDegrees() {
        return angle;
    }
    
    @Override
    public double getSkewInDegrees() {
        return skew;
    }

      /**
   * @return the change in inches for x, y value from starting odometry value to
   *         final point.
   */
  public Translation2d getFinalPoint() {
    double distanceMeters = Units.inchesToMeters(distance);
    double angleRadians = Math.toRadians(angle);
    return new Translation2d(distanceMeters * Math.cos(angleRadians) - Units.inchesToMeters(6), 
            distanceMeters * Math.sin(angleRadians));
  }

  /**
   * @return the change in inches for x, y value from starting odometry value to
   *         the waypoint.
   */
  public Translation2d getWaypoint() {
    double distanceMeters = Units.inchesToMeters(distance);
    double angleRadians = Math.toRadians(angle);
    return new Translation2d(distanceMeters * Math.cos(angleRadians) - Units.inchesToMeters(36), 
            distanceMeters * Math.sin(angleRadians));
  }
}
