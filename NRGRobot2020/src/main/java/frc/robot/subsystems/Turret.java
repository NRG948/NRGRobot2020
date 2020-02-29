package frc.robot.subsystems;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.NRGPreferences;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.StopTurretAnglePID;
import frc.robot.commands.TurnTurretToAngle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * Robot subsystem that controls the rotation of the shooter turret.
 */
public class Turret extends SubsystemBase {

  // TODO: min and max values need to be figured out; the values below are
  // fictious values.
  private static final double MIN_ENCODER_VALUE = 0;
  private static final double MAX_ENCODER_VALUE = 170;
  private static final double CAMERA_HORIZONTAL_CORRECTION_PRACTICE = -3;
  private static final double CAMERA_HORIZONTAL_CORRECTION_COMPETITION = 0;
  private double cameraHorizontalCorrection;

  private final Victor turretMotor = new Victor(TurretConstants.kTurretMotorPort); // Change back to victor when we get
                                                                                   // the actual robot
  private final Encoder turretEncoder = new Encoder(TurretConstants.kTurretEncoderPorts[0],
      TurretConstants.kTurretEncoderPorts[1]);
  private PIDController turretPIDController;
  private double maxPower;

  private boolean continuousPID = false;
  private double skewHorizontalAngle;

  private LimelightVision limelightVision;

  /**
   * Creates the Turret subsystem.
   */
  public Turret(LimelightVision limelightVision) {
    this.limelightVision = limelightVision;
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
   * @param power
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
  public void turretAnglePIDInit(double desiredAngleX, double maxPower, double tolerance, boolean continuousPID) {
    this.limelightVision.turnOnLed();
    this.maxPower = maxPower;

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
   * @param limelightAngleX from limelight is used to calculate power
   */
  public void turretAngleToExecute(double limelightAngleX) {
    double pidOutput = turretPIDController.calculate(limelightAngleX);
    double currentPower = MathUtil.clamp(pidOutput, -1.0, 1.0) * maxPower;
    rawTurret(currentPower);
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
      double currentAngle = limelightVision.getX();
      this.turretAngleToExecute(currentAngle);
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
    ShuffleboardLayout turretLayout = turretTab.getLayout("Turret", BuiltInLayouts.kList).withPosition(0, 0).withSize(2,
        5);
    turretLayout.add("Encoder", turretEncoder);
    turretLayout.addNumber("PID Position Error",
        () -> (turretPIDController != null) ? turretPIDController.getPositionError() : 0.0);
    turretLayout.addNumber("Raw Output", () -> (turretMotor.get()));
    turretLayout.addBoolean("ContinuousPID", () -> (continuousPID));
    turretLayout.addNumber("Limelight x", () -> (limelightVision.getX()));
    turretLayout.add("Stop angle PID", new StopTurretAnglePID(this));
    turretLayout.add("Turret turn to angle", new TurnTurretToAngle(this, 130));

    VideoSource processedVideo = new HttpCamera("limelight", "http://limelight.local:5800/stream/mjpg");

    turretTab.add("Processed Video", processedVideo).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0)
        .withSize(4, 3);
  }
}
