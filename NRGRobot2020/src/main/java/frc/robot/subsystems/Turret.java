package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.NRGPreferences;
import frc.robot.utilities.TargetSource;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.AutoTurret;
import frc.robot.commands.StopTurretAnglePID;
import frc.robot.commands.TurnTurretToAngle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.util.Units;

/**
 * Robot subsystem that controls the rotation of the shooter turret.
 */
public class Turret extends SubsystemBase {

  private static final double MIN_ENCODER_VALUE = 0;
  private static final double MAX_ENCODER_VALUE = 170;
  private static final double CAMERA_HORIZONTAL_CORRECTION_PRACTICE = -1.5;
  private static final double CAMERA_HORIZONTAL_CORRECTION_COMPETITION = 0;
  private static final Transform2d TURRET_POSITION_OFFSET = new Transform2d(
    new Translation2d(Units.inchesToMeters(-9.5), Units.inchesToMeters(-8.25)),
    new Rotation2d(0.0));
  private static final double TURRET_ANGLE_OFFSET = 51.0;
  private static final Translation2d POWERPORT_FIELD_POSITION = new Translation2d(0, Units.inchesToMeters(-94.66));
  private double cameraHorizontalCorrection;

  private final Victor turretMotor = new Victor(TurretConstants.kTurretMotorPort);
  private final Encoder turretEncoder = new Encoder(TurretConstants.kTurretEncoderPorts[0],
      TurretConstants.kTurretEncoderPorts[1]);
  private PIDController turretPIDController;
  private double maxPower;
  private double lastAngle = 0;

  private boolean continuousPID = false;
  private double skewHorizontalAngle;

  private LimelightVision limelightVision;
  private Drive drive;
  private TargetSource targetSource = TargetSource.NONE;

  /**
   * Creates the Turret subsystem.
   */
  public Turret(LimelightVision limelightVision, Drive drive) {
    this.limelightVision = limelightVision;
    this.drive = drive;
    turretMotor.setInverted(false);
    turretEncoder.setDistancePerPulse(0.027);
    cameraHorizontalCorrection = NRGPreferences.USING_PRACTICE_BOT.getValue()
      ? CAMERA_HORIZONTAL_CORRECTION_PRACTICE : CAMERA_HORIZONTAL_CORRECTION_COMPETITION;
  }

  public void resetHeading() {
    turretEncoder.reset();
  }

  /**
   * Passes power to turret with hard stop protection.
   * 
   * @param power turret motor power, + means counter-clockwise, - means clockwise
   * (because the encoder value increases counter-clockwise)
   */
  public void rawTurret(double power) {
    double encoderAngle = turretEncoder.getDistance();
    // Prevent the turret from turning past hard stops
    if (encoderAngle >= MAX_ENCODER_VALUE && power > 0 || encoderAngle < MIN_ENCODER_VALUE && power < 0) {
      power = 0;
    }
    turretMotor.set(power);
  }

  public double getHeading() {
    return turretEncoder.getDistance();
  }

  /**
   * Initializes PID controller for turret.
   * 
   * @param desiredAngleX
   * @param tolerance
   */
  public void turretAnglePIDInit(TargetSource targetSource, double desiredAngleX, double maxPower, double tolerance, boolean continuousPID) {
    this.maxPower = maxPower;
    this.targetSource = targetSource;
    if (targetSource == TargetSource.LIMELIGHT) {
      this.limelightVision.turnOnLed();
    }

    double kP = NRGPreferences.TURRET_P_TERM.getValue();
    double kI = NRGPreferences.TURRET_I_TERM.getValue();
    double kD = NRGPreferences.TURRET_D_TERM.getValue();

    this.turretPIDController = new PIDController(kP, kI, kD);
    this.turretPIDController.setSetpoint(desiredAngleX + cameraHorizontalCorrection + skewHorizontalAngle);
    this.turretPIDController.setTolerance(tolerance);

    this.continuousPID = continuousPID;
    System.out.println("Turret Init");
  }

  /**
   * Updates turret motor power based on output from the PID controller.
   * 
   * @param angle The current angle of the turret or angle to target from the limelight.
   */
  public void turretAngleToExecute(double angle) {
    double pidOutput = turretPIDController.calculate(angle);
    double currentPower = MathUtil.clamp(pidOutput, -1.0, 1.0) * maxPower;
    rawTurret(currentPower);
    lastAngle = angle;
  }

  /**
   * Returns whether the turret is within the tolerance of the setpoint.
   * 
   * @return true when turret position is on target
   */
  public boolean turretAngleOnTarget() {
    return this.turretPIDController.atSetpoint();
  }

  /**
   * Stops the turretMotor at the end of a turret command.
   */
  public void turretAngleEnd() {
    this.turretMotor.stopMotor();
    this.turretPIDController = null;
    continuousPID = false;
    this.limelightVision.turnOffLed();
    System.out.println("Turret End");
  }

  public void setHorizontalSkew(double skewAngle){
    this.skewHorizontalAngle = skewAngle;
  }

  @Override
  public void periodic() {
    if (continuousPID) {
      if (DriverStation.getInstance().isDisabled()) {
        this.turretAngleEnd();
      } else {
        double currentAngle = 0;

        if (this.targetSource == TargetSource.LIMELIGHT) {
          currentAngle = limelightVision.getX();
        } else if (this.targetSource == TargetSource.POSITION) {
          Translation2d position = drive.getPose().plus(TURRET_POSITION_OFFSET).getTranslation();
          currentAngle = Math.toDegrees(Math.atan2(
            position.getY()-POWERPORT_FIELD_POSITION.getY(), position.getX()-POWERPORT_FIELD_POSITION.getX()));
          currentAngle += drive.getHeading() - TURRET_ANGLE_OFFSET;
        }

        this.turretAngleToExecute(currentAngle);
      }
    }
  }

  /**
   * Initializes the Shuffleboard Tab that displays debug information about the
   * Turret subsystem.
   */
  public void initShuffleboard() {
    if (!NRGPreferences.SHUFFLEBOARD_TURRET_ENABLED.getValue()){
      return;
    }
    
    ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");
    ShuffleboardLayout turretLayout = turretTab.getLayout("Turret", BuiltInLayouts.kList)
      .withPosition(0, 0)
      .withSize(2, 5);
    turretLayout.add("Encoder", turretEncoder);
    turretLayout.addNumber("Angle", () -> lastAngle);
    turretLayout.addNumber("PID Position Error",
        () -> (turretPIDController != null) ? turretPIDController.getPositionError() : 0.0);
    turretLayout.addNumber("Raw Output", () -> turretMotor.get());
    turretLayout.addBoolean("ContinuousPID", () -> continuousPID);
    turretLayout.addNumber("Limelight x", () -> limelightVision.getX());

    ShuffleboardLayout controlLayout = turretTab.getLayout("Control", BuiltInLayouts.kList)
      .withPosition(6, 0)
      .withSize(2, 4);
    controlLayout.add("Stop angle PID", new StopTurretAnglePID(this));
    controlLayout.add("Turret turn to angle", new TurnTurretToAngle(this, 130));
    controlLayout.add("AutoTurret using Limelight", new AutoTurret(this).usingLimelight());
    controlLayout.add("AutoTurret using Position", new AutoTurret(this).usingPosition());

    VideoSource processedVideo = new HttpCamera("limelight", "http://limelight.local:5800/stream/mjpg");

    turretTab.add("Limelight", processedVideo).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0)
        .withSize(4, 3);
  }
}
