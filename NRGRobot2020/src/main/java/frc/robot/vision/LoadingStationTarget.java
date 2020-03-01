/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.utilities.NRGPreferences;

/**
 * Add your docs here.
 */
public class LoadingStationTarget implements VisionTarget{

    private double distance;
    private double skew;
    private double angle;

    public LoadingStationTarget(double distance, double angle, double skew) {
        this.distance = distance * NRGPreferences.LOADING_STATION_DISTANCE_FUDGE.getValue();
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
  public Translation2d getFinalPoint(double heading, double xOffset, double yOffset) {
    double distanceMeters = Units.inchesToMeters(distance);
    double angleRadians = Math.toRadians(angle + heading);
    return new Translation2d(distanceMeters * Math.cos(angleRadians) - Units.inchesToMeters(6) + xOffset, 
            distanceMeters * Math.sin(angleRadians) + yOffset);
  }

  /**
   * @return the change in inches for x, y value from starting odometry value to
   *         the waypoint.
   */
  public Translation2d getWaypoint(double heading, double xOffset, double yOffset) {
    double distanceMeters = Units.inchesToMeters(distance);
    double angleRadians = Math.toRadians(angle + heading);
    // We multiply the x-distance by 0.7 to create a waypoint, even if the robot is close to the target
    return new Translation2d(0.7 * (distanceMeters * Math.cos(angleRadians) - Units.inchesToMeters(6) + xOffset), 
            distanceMeters * Math.sin(angleRadians) + yOffset);
  }
}
