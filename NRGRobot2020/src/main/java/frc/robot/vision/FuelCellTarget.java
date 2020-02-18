package frc.robot.vision;

import frc.robot.vision.VisionTarget;

public class FuelCellTarget implements VisionTarget {
    private double distance;
    private double angle;

    public FuelCellTarget(double distance, double angle) {
        this.distance = distance;
        this.angle = angle;
    }

    @Override
    public double getDistanceInInches() {
        return this.distance;
    }

    @Override
    public double getAngleInDegrees() {
        return this.angle;
    }

    @Override
    public double getSkewInDegrees() {
        return 0;
    }
}
