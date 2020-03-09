package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.AutoDriveOnHeading;
import frc.robot.utilities.NRGPreferences;
import frc.robot.utilities.Observer;
import frc.robot.utilities.Subject;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.commands.FollowWaypoints;
import frc.robot.test.IMotorEncoderPair;

public class Drive extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  // Gyro Declaration
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  // Motor Declaration
  private WPI_VictorSPX rightMotor1 = new WPI_VictorSPX(DriveConstants.kRightMotor1Port);
  private WPI_VictorSPX rightMotor2 = new WPI_VictorSPX(DriveConstants.kRightMotor2Port);
  private WPI_VictorSPX leftMotor1 = new WPI_VictorSPX(DriveConstants.kLeftMotor1Port);
  private WPI_VictorSPX leftMotor2 = new WPI_VictorSPX(DriveConstants.kLeftMotor2Port);

  // The motors on the left side of the drive.
  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftMotor1, leftMotor2);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightMotor1, rightMotor2);

  // The robot's drivez
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
  private double lastWorldAccelX;
  private double lastWorldAccelY;
  private boolean detectCollisions = false;
  private int collisionCount;
  private Subject<Drive> observableCollision = new Subject<Drive>();

  public Drive() {
    leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    leftMotors.setInverted(false);
    rightMotors.setInverted(false);
    resetHeading();
    resetEncoders();
  }

  /**
   * Basic Tank Drive method for drive subsystem, takes direct inputs for left and
   * right sides.
   * 
   * @param leftPower    value from -1 to 1 set to left motor group. + is forward.
   * @param rightPower   value from -1 to 1 set to right motor group. + is
   *                     forward.
   * @param squareInputs squares motor inputs if true
   */
  public void tankDrive(double leftPower, double rightPower, boolean squareInputs) {
    diffDrive.tankDrive(leftPower, rightPower, squareInputs);
  }

  /**
   * Tank Drive with volatge inputs instead of power inputs.
   * 
   * Voltage inputs allows more control over motors.
   * 
   * @param leftVolts  Value from -12 to 12 volts set to left motor group. + is
   *                   forward.
   * @param rightVolts Value from -12 to 12 volts set to right motor group. + is
   *                   forward.
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
    diffDrive.feed();
  }

  /**
   * Drive method that uses 2 inputs on the x and z axis.
   * 
   * Allows drive with singuler joystick/controller.
   * 
   * @param xPower   The robot's speed along the X axis [-1.0..1.0]. + is forward
   * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0]. + is
   *                 Clockwise.
   */
  public void arcadeDrive(double xPower, double rotation) {
    diffDrive.setDeadband(0);
    diffDrive.arcadeDrive(xPower, rotation, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Get my gyro angle. We are negating the value because gyros return positive
    // values as the robot turns clockwise. This is not standard convention that is
    // used by the WPILib classes.
    Rotation2d gyroAngle = Rotation2d.fromDegrees(-navx.getAngle());

    // Update the pose
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();

    odometry.update(gyroAngle, leftDistance, rightDistance);

    detectCollisions();
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void resetHeading() {
    navx.reset();
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
    odometry.resetPosition(pose, pose.getRotation());
  }

  /**
   * Resets Encoders
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(navx.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  /**
   * returns the continuous heading of the robot
   * 
   * @return the continuous heading of the robot
   */
  public double getHeadingContinuous() {
    return navx.getAngle() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
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
   * @param heading Angle we wish to drive.
   */
  public void driveOnHeadingInit(double currentHeading) {
    double p = NRGPreferences.DRIVE_P_TERM.getValue();
    double i = NRGPreferences.DRIVE_I_TERM.getValue();
    double d = NRGPreferences.DRIVE_D_TERM.getValue();
    this.drivePIDController = new PIDController(p, i, d);
    this.drivePIDController.setSetpoint(currentHeading);
    this.drivePIDController.setTolerance(0);
    setCurrentHeading(currentHeading);
  }
  
  public void driveOnHeadingExecute(double power) {
    double powerDelta = this.drivePIDController.calculate(getHeadingContinuous());
    // if (Math.signum(powerDelta) != Math.signum(power)) {
      // this.tankDrive(power + powerDelta, power, false);
      // } else {
        // this.tankDrive(power, power - powerDelta, false);
        // }
        powerDelta = MathUtil.clamp(powerDelta, -Math.abs(power), Math.abs(power));

    this.arcadeDrive(power, -powerDelta);
    
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

  /**
   * Sets the current heading of the robot.
   * 
   * @param currentHeading Current heading of the robot.
   */
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
    double p = NRGPreferences.TURN_P_TERM.getValue();
    double i = NRGPreferences.TURN_I_TERM.getValue();
    double d = NRGPreferences.TURN_D_TERM.getValue();
    this.turnPIDController = new PIDController(p, i, d);
    this.turnPIDController.setSetpoint(desiredHeading);
    this.turnPIDController.setTolerance(tolerance);
    this.turnSquareInputs = areTurnInputsSquared();
  }
  
  /**
   * Returns if Turn inputs are squared
   * 
   * @return boolean TURN_SQUARE_INPUTS
   */
  public boolean areTurnInputsSquared() {
    return NRGPreferences.TURN_SQUARE_INPUTS.getValue();
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
    double currentPower = this.turnPIDController.calculate(getHeadingContinuous()) * maxPower;
    if (useBothSides) {
      this.arcadeDrive(0, -currentPower);
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
  
  /**
   * Adds a Shuffleboard tab for the drive subsystem.
   */
  public void addShuffleBoardTab() {
    if (!NRGPreferences.SHUFFLEBOARD_DRIVE_ENABLED.getValue()){
      return;
    }
    
    ShuffleboardTab driveTab = Shuffleboard.getTab("Drive");

    // Add test buttons to a layout in the tab
    ShuffleboardLayout commandsLayout = driveTab.getLayout("Test", BuiltInLayouts.kList)
    .withPosition(0, 0)
    .withSize(2, 3);
    
    commandsLayout.add("Turn to 90", new AutoTurnToHeading(this).withMaxPower(0.35).toHeading(90));
    commandsLayout.add("Turn to -90", new AutoTurnToHeading(this).withMaxPower(0.35).toHeading(-90));
    commandsLayout.add("Drive 1 meter", new AutoDriveOnHeading(this).withMaxPower(0.5).forMeters(1));
    commandsLayout.add("Drive 3 meters", new AutoDriveOnHeading(this).withMaxPower(0.5).forMeters(3));
    commandsLayout.add("Follow S Curve",
      new InstantCommand(() -> { this.resetHeading(); this.resetOdometry(new Pose2d(0, 0, new Rotation2d())); })
      .andThen( new FollowWaypoints(this, new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(1, -1), new Translation2d(2, 1)), new Pose2d(3, 0, new Rotation2d(0)), false)));
          
    // Add the DifferentialDrive object and encoders to a list layout in the tab.
    ShuffleboardLayout diffDriveLayout = driveTab.getLayout("Base", BuiltInLayouts.kList).
    withPosition(2, 0).
      withSize(4, 5);
      
      diffDriveLayout.add("Differential Drive", diffDrive).withWidget(BuiltInWidgets.kDifferentialDrive);
      diffDriveLayout.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kEncoder);
      diffDriveLayout.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kEncoder);
      
      // Add the odometry to a layout in the tab.
      ShuffleboardLayout positionLayout = driveTab.getLayout("Position", BuiltInLayouts.kList).
      withPosition(6, 0).
      withSize(2, 2);
      
      positionLayout.addNumber("X", () -> getPose().getTranslation().getX());
      positionLayout.addNumber("Y", () -> getPose().getTranslation().getY());
      positionLayout.addNumber("Heading", () -> getHeadingContinuous());
      
      // Add collision detection to a layout tab
      ShuffleboardLayout collisionLayout = driveTab.getLayout("Collision", BuiltInLayouts.kList)
      .withPosition(6, 2)
      .withSize(2, 2);
      
    collisionLayout.addNumber("Collision Count", () -> this.collisionCount);
    collisionLayout.add("Enable", new InstantCommand(() -> this.setDetectCollisions(true)));
    collisionLayout.add("Disable", new InstantCommand(() -> this.setDetectCollisions(false)));
  }

  private boolean collisionDetected() {
    double currentWorldAccelX = this.navx.getWorldLinearAccelX();
    double currentWorldAccelY = this.navx.getWorldLinearAccelY();
    double currentJerkX = currentWorldAccelX - this.lastWorldAccelX;
    double currentJerkY = currentWorldAccelY - this.lastWorldAccelY;
    double currentJerk = Math.sqrt(currentJerkX * currentJerkX + currentJerkY * currentJerkY);
    return currentJerk >= NRGPreferences.DRIVE_COLLISION_THRESHOLD.getValue();
  }
  
  private void detectCollisions() {
    if (this.detectCollisions && collisionDetected()) {
      this.observableCollision.notify();
      ++this.collisionCount;
    }
  }
  
  public void setDetectCollisions(boolean detectCollisions) {
    this.detectCollisions = detectCollisions;
  }
  
  public void onCollisionDetected(Observer<Drive> collisionObserver) {
    this.observableCollision.addObserver(collisionObserver);
  }

  public int getCollisionCount() {
    return this.collisionCount;
  }

  /**
  public class MotorEncoderPair implements IMotorEncoderPair {
    private WPI_VictorSPX motor;
    private Encoder encoder;

    public MotorEncoderPair(WPI_VictorSPX motor, Encoder encoder){
      this.motor = motor;
      this.encoder = encoder;
    }

    public void setMotor(double power){
      motor.set(power);
    }

    public double getEncoder(){
      return encoder.getDistance();
    }
  }
  */
}
