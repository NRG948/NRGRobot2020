/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;

/**
 * Add your docs here.
 */
public class Target { 
    /**
     * To calculate distance, we might only need
     * the Y values. To calculate angle/skew,
     * we need both.
     * */
    private Point minX;
    private Point maxX;
    private Point minY;
    private Point maxY;
    private boolean isCameraInverted = false;
    private boolean empty = false;

    public Target() {
        this.empty = true;
    }

    public Target(MatOfPoint mat, boolean isCameraInverted) {
        this.isCameraInverted = isCameraInverted;
        Point[] points = mat.toArray();
        Point first = points[0];
        if (isCameraInverted) {
          first.x = -first.x;
        } else {
          first.y = -first.y;
        }
        minX = first;
        maxX = first;
        minY = first;
        maxY = first;
    
        for (int i = 1; i < points.length; i++) {
          Point point = points[i];
          if (isCameraInverted) {
            point.x = -point.x;
          } else {
            point.y = -point.y;
          }
          if (point.x < minX.x) {
            minX = point;
          }
          if (point.y < minY.y) {
            minY = point;
          }
          if (point.x > maxX.x) {
            maxX = point;
          }
          if (point.y > maxY.y) {
            maxY = point;
          }
        }
    }

    public String toString() {
        return "minX: " + minX.x + " maxX: " + maxX.x + " minY: " + 
                minY.y + " maxY " + maxY.y;
    }

    public Point getMinX() {
        return this.minX;
    }
    
    public Point getMinY() {
        return this.minY;
    }
    
    public Point getMaxX() {
        return this.maxX;
    }
    
    public Point getMaxY() {
        return this.maxY;
    }  
    
    // Do I need this?
    public Point getCenter() {
        return new Point((getMinX().x + getMaxX().x) / 2, (getMinY().y + getMaxY().y) / 2);
    }

    // Do I need this?
    public MatOfPoint toMatOfPoint() {
        Point adjMinX = new Point();
        Point adjMinY = new Point();
        Point adjMaxX = new Point();
        Point adjMaxY = new Point();
        if (isCameraInverted) {
          adjMinX.x = -this.minX.x;
          adjMinX.y = this.minX.y;
          adjMaxX.x = -this.maxX.x;
          adjMaxX.y = this.maxX.y;
          adjMinY.x = -this.minY.x;
          adjMinY.y = this.minY.y;
          adjMaxY.x = -this.maxY.x;
          adjMaxY.y = this.maxY.y;
        } else {
          adjMinX.x = this.minX.x;
          adjMinX.y = -this.minX.y;
          adjMaxX.x = this.maxX.x;
          adjMaxX.y = -this.maxX.y;
          adjMinY.x = this.minY.x;
          adjMinY.y = -this.minY.y;
          adjMaxY.x = this.maxY.x;
          adjMaxY.y = -this.maxY.y;
        }
        return new MatOfPoint(adjMinX, adjMinY, adjMaxX, adjMaxY);
    }
    
    public boolean getIsCameraInverted() {
        return this.isCameraInverted;
    }

    public boolean isEmpty() {
        return this.empty;
    }
}
