/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.awt.Point;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.time.Instant;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.NRGPreferences;

/**
 * Add your docs here.
 */
public class LoadingVisionTarget {
    private static final double DISTANCE_THRESHOLD = 100.0;
    private static final double HALF_IMAGE_FOV = (Math.atan(36.0 / 57.125));
    private static final double DEFAULT_HALF_IMAGE_WIDTH = 480 / 2;
    private static final double TARGET_WIDTH_INCHES = 7.0;

    private Target target = new Target();
    private double imageCenterX;
    private int getCount = Integer.MAX_VALUE;
    private Gson gson = new Gson();
    private String targetJson;

    public LoadingVisionTarget() {
        File dir = new File(Filesystem.getOperatingDirectory(), "targets");
        dir.mkdirs();
    }

    public void update() {
        int newGetCount = (int) SmartDashboard.getNumber("Vision/getCount", this.getCount);

        if (this.getCount != newGetCount) {
            this.getCount = newGetCount;
            this.imageCenterX = SmartDashboard.getNumber("Vision/imageCenterX", DEFAULT_HALF_IMAGE_WIDTH);
            this.targetJson = SmartDashboard.getString("Vision/target", "");
            Target newTarget = gson.fromJson(targetJson, Target.class);
            this.target = newTarget;
        }
    }  

    public int getCount() {
        return this.getCount;
    }

    public boolean hasTargets() {
        return !this.target.isEmpty() && getDistanceToTarget() < DISTANCE_THRESHOLD;
    }
    
    private Target getDesiredTarget() {
        return this.target;
    }

    //TODO: create getSkew in order to know calculate the path the robot
    // should take to end up facing the loading station high port head-on

    public double getAngleToTarget() {
        double centerX = this.target.getCenter().x;
        double deltaX = centerX - imageCenterX;
        return Math.toDegrees(Math.atan2(deltaX, imageCenterX / Math.tan(HALF_IMAGE_FOV)))
            * NRGPreferences.NumberPrefs.CAMERA_ANGLE_SCALE.getValue();
    }

    public double getDistanceToTarget() {
        Target desiredTarget = getDesiredTarget();
        double targetWidth = (desiredTarget.getMinX().x - desiredTarget.getMaxX().x);
        double distance = (TARGET_WIDTH_INCHES * imageCenterX / (targetWidth * Math.tan(HALF_IMAGE_FOV)))
            * NRGPreferences.NumberPrefs.CAMERA_DISTANCE_SCALE.getValue();
        return distance / Math.cos(Math.toRadians(this.getAngleToTarget()));
    } 

    public void saveTargets() {
        try {
            String filename = "targets/" + Instant.now().toString() + ".json";
            File file = new File(Filesystem.getOperatingDirectory(), filename);
            try (PrintStream stream = new PrintStream(file)) {
                stream.println(targetJson);
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
}
