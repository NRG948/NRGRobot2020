/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private Victor driveFrontRightMotor = new Victor(9);
  private Victor driveMiddleRightMotor = new Victor(10);
  private Victor driveBackRightMotor = new Victor(2);
  private Victor driveFrontLeftMotor = new Victor(3);
  private Victor driveMiddleLeftMotor = new Victor(4);
  private Victor driveBackLeftMotor = new Victor(5);

  // The motors on the left side of the drive.
  private final SpeedControllerGroup leftMotors =
    new SpeedControllerGroup(driveFrontLeftMotor, driveMiddleLeftMotor, driveBackLeftMotor);
  // The motors on the right side of the drive.
  private final SpeedControllerGroup rightMotors =
    new SpeedControllerGroup(driveFrontRightMotor, driveMiddleRightMotor, driveBackRightMotor);  
  // The robot's drive
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  // The odometry (position-tracker)
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(
      Rotation2d.fromDegrees(getHeading()),
      new Pose2d(0.0, 0.0, new Rotation2d()));
  // The left-side drive encoder
  private final Encoder leftEncoder =
      new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1],
      DriveConstants.kLeftEncoderReversed);
// The right-side drive encoder
private final Encoder rightEncoder =
      new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1],
      DriveConstants.kRightEncoderReversed);
  
  public Drive() {

  }
  public void tankDrive(double leftPower, double rightPower){
    leftMotors.set(leftPower);
    rightMotors.set(rightPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    Rotation2d gyroAngle = Rotation2d.fromDegrees(-Robot.navx.getAngle());
    // Update the pose
    Pose2d pose = odometry.update(gyroAngle, leftEncoder.getDistance(), rightEncoder.getDistance());
    SmartDashboard.putString("position", pose.toString());
  }

  /**
  * Zeroes the heading of the robot.
  */
  public void zeroHeading() {
    Robot.navx.reset();
  }

  /**
   * 
   * @return The x,y position of the robot
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
  }
  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return rightEncoder;
  }

  /**
  * Resets the odometry to the specified pose.
  *
  * @param pose The pose to which to set the odometry.
  */
  public void resetOdometry(Pose2d pose) {
  resetEncoders();
  odometry.resetPosition(pose, 
      new Rotation2d());
  }

  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
  * Returns the heading of the robot.
  *
  * @return the robot's heading in degrees, from 180 to 180
  */
  public double getHeading() {
    return Math.IEEEremainder(Robot.navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }  
  /**
  * Returns the turn rate of the robot.
  *
  * @return The turn rate of the robot, in degrees per second
  */
  public double getTurnRate() {
    return Robot.navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
