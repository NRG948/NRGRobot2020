package frc.robot.utilities;

import frc.robot.subsystems.LimelightVision;

/**
 * Add your docs here.
 */
public class CalculateValuesInnerTarget {
    private final LimelightVision limelight;
    private double width;
    private double height;
    private double skew;
    private double distanceToOuter;
    private double distanceToInner;
    private double xOffsetAngle;
    private double yOffsetAngle;
    // k is the constant that we multiply the inverse cosine by to get a more accurate angle
    private double k = 1.0;

    public CalculateValuesInnerTarget(LimelightVision limelight) {
        this.limelight = limelight;
    }

    public void update() {
        setSkew();
        setDistanceToOuter();
        setDistanceToInner();
        setXOffsetAngle();
        setYOffsetAngle();
    }

    public void setWidth() {
        width = limelight.getWidth();
    }

    public double getWidth() {
        return width;
    }

    public void setHeight() {
        height = limelight.getHeight();
    }

    public double getHeight() {
        return height;
    }

    public void setSkew() {
        skew = Math.acos(k * width * Math.sqrt(3) / (4 * height));
    }

    public double getSkew() {
        return skew;
    }

    public void setDistanceToOuter() {
        distanceToOuter = limelight.getDistance() * NRGPreferences.CAMERA_DISTANCE_SCALE.getValue(); // should be in inches
    }

    public double getDistanceToOuter() {
        return distanceToOuter;
    }

    public void setDistanceToInner() {
        distanceToInner = Math.sqrt(Math.pow(29.25, 2.0) + Math.pow(distanceToOuter, 2.0) - 
                2 * 29.25 * distanceToOuter * Math.cos(180 - skew));
    }

    public double getDistanceToInner() {
        return distanceToInner;
    }

    public void setXOffsetAngle() {
        xOffsetAngle = Math.acos((Math.pow(29.25, 2.0) - Math.pow(distanceToInner, 2.0) - Math.pow(distanceToOuter, 2.0)) 
                / (-2.0 * distanceToInner * distanceToOuter));
    } 

    public double getXOffsetAngle() {
        return xOffsetAngle;
    }
    
    public void setYOffsetAngle() {
        yOffsetAngle = Math.acos((Math.pow(29.25, 2.0) - Math.pow(distanceToOuter, 2.0) - Math.pow(distanceToInner, 2.0)) 
                / (-2.0 * distanceToInner * distanceToOuter));
    }
    
    public double getYOffsetAngle() {
        return yOffsetAngle;
    }
}
