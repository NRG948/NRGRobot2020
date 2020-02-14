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

    private double distance;
    private double skew;
    private double angle;

    public LoadingStationTarget(double distance, double angle, double skew){
        this.distance = distance;
        this.angle = angle;
        this.skew = skew;
    }


    public double getDistance() {
        return distance;
    }
    
    public double getAngleToTarget() {
        return angle;
    }
    
    public double getSkew() {
        return skew;
    }
}
