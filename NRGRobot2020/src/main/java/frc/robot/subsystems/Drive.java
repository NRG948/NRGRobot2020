/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.utilities.NRGPreferences;

public class Drive extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(DriveConstants.kRightMotor1Port);
  private WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);
  private WPI_VictorSPX rightMotor3 = new WPI_VictorSPX(DriveConstants.kRightMotor3Port);
  private WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(DriveConstants.kLeftMotor1Port);
  private WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);
  private WPI_VictorSPX leftMotor3 = new WPI_VictorSPX(DriveConstants.kLeftMotor3Port);

  // The motors on the left side of the drive.
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2,
      leftMotor3);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2,
      rightMotor3);

  // The robot's drive
  private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

  // The odometry (position-tracker)
  private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()),
      new Pose2d(0.0, 0.0, new Rotation2d()));

  // The left-side drive encoder
  private final Encoder leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0],
      DriveConstants.kLeftEncoderPorts[1], DriveConstants.kLeftEncoderReversed);

  // The right-side drive encoder
  private final Encoder rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0],
      DriveConstants.kRightEncoderPorts[1], DriveConstants.kRightEncoderReversed);

  // Variables for DriveDistance
  private double currentHeading = 0;
  private PIDController drivePIDController;
  private PIDController turnPIDController;
  private boolean turnSquareInputs;

  public Drive() {
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    leftMotors.setInverted(true);
    rightMotors.setInverted(true);
  }

  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    diffDrive.tankDrive(leftPower, rightPower, squareInputs);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    diffDrive.feed();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    Rotation2d gyroAngle = Rotation2d.fromDegrees(-Robot.navx.getAngle());
    // Update the pose
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    Pose2d pose = odometry.update(gyroAngle, leftDistance, rightDistance);
    
    SmartDashboard.putNumber("Drive/Left Distance", leftDistance);
    SmartDashboard.putNumber("Drive/Right Distance", rightDistance);
    SmartDashboard.putString("Drive/position", pose.toString());
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
    odometry.resetPosition(pose, new Rotation2d());
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

  /**
   * 
   * @param xOrigin
   * @param yOrigin
   * @return The distance between the current position and the original position
   */

  public double calculateDistance(double xOrigin, double yOrigin) {
    return getPose().getTranslation().getDistance(new Translation2d(xOrigin, yOrigin));
  }

  /**
   * We are getting our current heading and putting it into driveOnHeadingInit to
   * adjust our current heading
   * 
   * @param heading
   */
  public void driveOnHeadingInit(double currentHeading) {
    double p = NRGPreferences.NumberPrefs.DRIVE_P_TERM.getValue();
    double i = NRGPreferences.NumberPrefs.DRIVE_I_TERM.getValue();
    double d = NRGPreferences.NumberPrefs.DRIVE_D_TERM.getValue();
    this.drivePIDController = new PIDController(p, i, d);
    this.drivePIDController.setSetpoint(getHeading());
    this.drivePIDController.setTolerance(0);
    setCurrentHeading(currentHeading);
  }

  public void driveOnHeadingExecute(double power) {
    double powerDelta = this.drivePIDController.calculate(Robot.navx.getAngle());
    if (Math.signum(powerDelta) != Math.signum(power)) {
      this.tankDrive(power + powerDelta, power, false);
    } else {
      this.tankDrive(power, power - powerDelta, false);
    }
    SmartDashboard.putNumber("Drive/driveOnHeading/PIDOutput", powerDelta);
    SmartDashboard.putNumber("Drive/driveOnHeading/PIDError", this.drivePIDController.getPositionError());
    SmartDashboard.putNumber("Drive/driveOnHeading/Setpoint", this.drivePIDController.getSetpoint());
  }

  /**
   * Stop driving
   */
  public void driveOnHeadingEnd() {
    this.diffDrive.stopMotor();
    this.drivePIDController = null;
  }

  public void setCurrentHeading(double currentHeading) {
    this.currentHeading = currentHeading;
  }

  /**
   * Performs all initialization for performing a robot's turn using a PID
   * controller.
   * 
   * @param desiredHeading the desired pose the robot should be at
   * @param tolerance
   */
  public void turnToHeadingInit(double desiredHeading, double tolerance) {
    double p = NRGPreferences.NumberPrefs.TURN_P_TERM.getValue();
    double i = NRGPreferences.NumberPrefs.TURN_I_TERM.getValue();
    double d = NRGPreferences.NumberPrefs.TURN_D_TERM.getValue();
    this.turnPIDController = new PIDController(p, i, d);
    this.turnPIDController.setSetpoint(desiredHeading);
    this.turnPIDController.setTolerance(tolerance);
    this.turnSquareInputs = areTurnInputsSquared();
  }

  public boolean areTurnInputsSquared() {
    return NRGPreferences.BooleanPrefs.TURN_SQUARE_INPUTS.getValue();
  }

  /**
   * Executes the robot turn by updating the motor values as needed every 20 ms.
   * 
   * @param maxPower maximum motor power used during the turn
   */
  public void turnToHeadingExecute(double maxPower) {
    turnToHeadingExecute(maxPower, true, true);
  }

  /**
   * Executes the robot turn by updating the motor values as needed every 20 ms.
   * 
   * @param maxPower     maximum motor power used during the turn
   * @param useBothSides if true, use both motors in opposite directions
   * @param forward      only used for one sided turns; if true robot pivots
   *                     forward, otherwise pivots back
   */
  public void turnToHeadingExecute(double maxPower, boolean useBothSides, boolean forward) {
    double currentPower = this.turnPIDController.calculate(Robot.navx.getAngle()) * maxPower;
    if (useBothSides) {
      this.tankDrive(currentPower, -currentPower, this.turnSquareInputs);
    } else {
      double leftPower;
      double rightPower;

      if (forward) {
        leftPower = currentPower > 0 ? currentPower : 0;
        rightPower = currentPower < 0 ? -currentPower : 0;
      } else {
        leftPower = currentPower < 0 ? currentPower : 0;
        rightPower = currentPower > 0 ? -currentPower : 0;
      }
      tankDrive(leftPower, rightPower, this.turnSquareInputs);
    }
  }

  /**
   * Determine whether the turn is finished or not.
   * 
   * @return true, if the turn is finished.
   */
  public boolean turnToHeadingOnTarget() {
    return this.turnPIDController.atSetpoint();
  }

  /**
   * Ends the turn command by shutting off the motors and disabling the PID
   * controllers.
   */
  public void turnToHeadingEnd() {
    this.diffDrive.stopMotor();
    this.turnPIDController = null;
  }
}
