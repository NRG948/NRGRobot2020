package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.NRGPreferences;
import frc.robot.Constants.TurretConstants;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

/**
 * Robot subsystem that controls the rotation of the shooter turret.
 */
public class Turret extends SubsystemBase {

  // TODO: min and max values need to be figured out; the values below are fictious values.
  private static final double MIN_ENCODER_VALUE = 0;
  private static final double MAX_ENCODER_VALUE = 4900;

  private final Victor turretMotor = new Victor(TurretConstants.kTurretMotorPort); // Change back to victor when we get the actual robot
  private final Encoder turretEncoder = new Encoder(TurretConstants.kTurretEncoderPorts[0], TurretConstants.kTurretEncoderPorts[1]);
  private PIDController turretPIDController;
  private double maxPower;

  private SimpleWidget turretPidErrorWidget;
  private SimpleWidget turretRawOutputWidget;
  private SimpleWidget turretPIDEnabledWidget;
  private boolean pidEnabled = false;

  private LimelightVision limelightVision;

  /**
   * Creates the Turret subsystem.
   */
  public Turret(LimelightVision limelightVision) {
    this.limelightVision = limelightVision;
    turretMotor.setInverted(false);
    turretEncoder.setDistancePerPulse(0.027);
  }

  /**
   * Initializes PID controller for turret.
   * 
   * @param desiredAngleX
   * @param tolerance
   */
  public void turretAnglePIDInit(double desiredAngleX, double maxPower, double tolerance) {
    this.maxPower = maxPower;

    double kP = NRGPreferences.TURRET_P_TERM.getValue();
    double kI = NRGPreferences.TURRET_I_TERM.getValue();
    double kD = NRGPreferences.TURRET_D_TERM.getValue();

    this.turretPIDController = new PIDController(kP, kI, kD);
    this.turretPIDController.setSetpoint(desiredAngleX);
    this.turretPIDController.setTolerance(tolerance);

    pidEnabled = true;
    System.out.println("Turret Init");
  }

  /**
   * Updates turret motor power based on output from the PID controller.
   * 
   * @param limelightAngleX from limelight is used to calculate power
   */
  public void turretAngleToExecute(double limelightAngleX) {
    turretPidErrorWidget.getEntry().setDouble(turretPIDController.getPositionError());
    double currentPower = this.turretPIDController.calculate(limelightAngleX) * maxPower;
    SmartDashboard.putNumber("Turret/power", this.turretPIDController.calculate(limelightAngleX));
    rawTurret(currentPower);
  }

  /**
   * Passes power to turret with hard stop protection.
   * @param power
   */
  public void rawTurret(double power){
    int encoderTicks = turretEncoder.get();
    //Prevent the turret from turning past hard stops
    if (encoderTicks >= MAX_ENCODER_VALUE && power > 0 || encoderTicks < MIN_ENCODER_VALUE && power < 0){
      power = 0;
    }
    turretRawOutputWidget.getEntry().setDouble(power);
    turretMotor.set(power);
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
    this.turretMotor.set(0);
    this.turretPIDController = null;
    pidEnabled = false;
  }

  @Override
  public void periodic() {
    turretPIDEnabledWidget.getEntry().setBoolean(pidEnabled);
    System.out.println(pidEnabled);
    System.out.println("x: "+limelightVision.getX());
    if (pidEnabled) {
      double currentAngle = limelightVision.getX();
      turretAngleToExecute(currentAngle);
    }
  }

  /** Initializes the Shuffleboard Tab that displays debug information about the Turret subsystem. */
  public void initShuffleboard(){
    ShuffleboardTab turretTab = Shuffleboard.getTab("Turret");
    ShuffleboardLayout turretLayout = turretTab.getLayout("Turret", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    turretLayout.add("Encoder", turretEncoder);
    turretPidErrorWidget = turretLayout.add("PID Position Error", 0.0);
    turretRawOutputWidget = turretLayout.add("Raw Output", 0.0);
    turretPIDEnabledWidget = turretLayout.add("PIDEnabled", false);
  }
}
