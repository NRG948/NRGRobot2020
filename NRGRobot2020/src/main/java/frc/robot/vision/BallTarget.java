/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.util.Units;

public class BallTarget {
    private static final double TARGET_WIDTH_INCHES = 7.1;
    private static final double HALF_IMAGE_FOV = (Math.atan(36.0 / 57.125));
    private Point center;
    private double diameterInPixels;
    private static final double IMAGE_CENTER_X = 640/2;
    public BallTarget(Point center, double diameterInPixels) {
        this.center = center;
        this.diameterInPixels = diameterInPixels;
    }
    public double distanceToTarget(){
        double distance = (TARGET_WIDTH_INCHES * IMAGE_CENTER_X / (diameterInPixels * Math.tan(HALF_IMAGE_FOV)));
        return Units.inchesToMeters(distance / Math.cos(Math.toRadians(this.getAngleToTarget())));
    }
    public double getAngleToTarget() {
        double deltaX = center.x - IMAGE_CENTER_X;
        return Math.toDegrees(Math.atan2(deltaX, IMAGE_CENTER_X / Math.tan(HALF_IMAGE_FOV)));
    }
}
