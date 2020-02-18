package frc.robot.vision;
public interface VisionTarget{
  /**
   * returns the distance to the target
   * @return the distance to target in inches
   */
  public double getDistanceInInches();
  /**
   * returns the angle to the target
   * @return the angle to target in degrees
   */
  public double getAngleInDegrees();
  /**
   * returns the skew of the target
   * @return the skew of the target in degrees
   */
  public double getSkewInDegrees();
}
