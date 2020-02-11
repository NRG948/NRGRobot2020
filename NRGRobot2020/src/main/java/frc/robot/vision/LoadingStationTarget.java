/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import frc.robot.subsystems.RaspberryPiVision;

/**
 * Add your docs here.
 */
public class LoadingStationTarget {
    private final double IMAGE_CENTER_X = 640.0 / 2;
    private final double HALF_IMAGE_FOV = Math.atan(36.0 / 57.125);

    private double distance;
    private double offsetX;
    private double skew;
    public LoadingStationTarget(final RaspberryPiVision raspPi) {
        distance = raspPi.getLoadingDistance();
        offsetX = raspPi.getLoadingOffsetX();
        skew = raspPi.getLoadingSkew();
    }

    public double getDistance() {
        return distance;
    }
    
    public double getAngleToTarget() {
        double deltaX = offsetX;
        return Math.toDegrees(Math.atan2(deltaX, IMAGE_CENTER_X / Math.tan(HALF_IMAGE_FOV)));
    }
    
    public double getSkew() {
        return skew;
    }
}
