package frc.robot.vision;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.util.Units;

public class FuelCellTarget {
    private double distance;
    private double angle;
    public FuelCellTarget(double distance, double angle) {
        this.distance = distance;
        this.angle = angle;
    }
    public double getDistanceToTarget(){
        return this.distance;
    }
    public double getAngleToTarget() {
        return this.angle;
    }
}
