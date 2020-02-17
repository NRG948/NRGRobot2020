package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.NRGPreferences;

public class AutoDriveOnHeading extends CommandBase {
  private final Drive drive;
  private double xOrigin;// a class variable
  private double yOrigin;
  private double distance;// this is a constant
  private double maxPower;
  private boolean stopMotors;

  private boolean useRobotHeading = true;
  private double heading = 0;
  /**
   * Creates a new DriveStraightDistance.
   */
  public AutoDriveOnHeading(final Drive subsystem) {
    this.drive = subsystem;
    this.maxPower = NRGPreferences.DRIVE_STRAIGHT_MAXPOWER.getValue();
    this.distance = 0;
    this.heading = this.drive.getHeading();
  }

  /**
   * Sets the max power to drive the robot
   * @param maxPower max power
   * @return this
   */
  public AutoDriveOnHeading withMaxPower(double maxPower){
    this.maxPower = maxPower;
    return this;
  }

  /**
   * Sets the distance to drive in meters
   * @param distance distance
   * @return this
   */
  public AutoDriveOnHeading forMeters(double distance){
    this.distance = distance;
    return this;
  }

  /**
   * Sets the distance to drive in inches
   * @param distanceInInches distance in inches
   * @return this
   */
  public AutoDriveOnHeading forInches(double distanceInInches){
    this.distance = Units.inchesToMeters(distanceInInches);
    return this;
  }

  /**
   * Sets the distance to drive in feet
   * @param distanceInFeet distance in feet
   * @return this
   */
  public AutoDriveOnHeading forFeet(double distanceInFeet){
    this.distance = Units.feetToMeters(distanceInFeet);
    return this;
  }

  /**
   * Sets the heading to drive on
   * @param heading heading
   * @return this
   */
  public AutoDriveOnHeading onHeading(double heading){
    this.heading = heading;
    this.useRobotHeading = false;
    return this;
  }
  
  // Called when the command
  @Override
  public void initialize() { 
    if (useRobotHeading) {
      this.heading = drive.getHeadingContinuous();
    }
    System.out.println("DriveStraightDistance Init heading: " + heading + " distance: " + distance);
    this.xOrigin = this.drive.getPose().getTranslation().getX(); // gets our current X position from the poistion tracker command
    this.yOrigin = this.drive.getPose().getTranslation().getY(); // gets our current Y position from the poistion tracker command
    this.drive.driveOnHeadingInit(this.heading); // We are getting our current heading and putting it into
                                            // driveOnHeadingInit to adjust our current heading
    System.out.println("AutoDriveOnHeading init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    this.drive.driveOnHeadingExecute(this.maxPower); // excuting the command and puts in the maximum power that the
    // robot is gonna run on  
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    if (stopMotors) {
      this.drive.driveOnHeadingEnd();
    }
    System.out.println(String.format("DriveStraightDistance End x:%.1f y:%.1f", this.drive.getPose().getTranslation().getX(),
        this.drive.getPose().getTranslation().getY()));
    // terminated the command as the robot has reached the distance that needs to be
    // traveled or if it needs to be interrupted
    System.out.println("AutoDriveOnHeading end");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.drive.calculateDistance(this.xOrigin, this.yOrigin) >= this.distance;
  }// this calculates the distance that we have traveled from origin in order to
   // figure out if the command
   // needs to be terminated or not and returns true or false true being it needs
   // to be terminated and false being it needs to be continued
}
