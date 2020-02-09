/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.NRGPreferences;
import frc.robot.Constants.TurretConstants;

/**
 * Robot subsystem that controls the rotation of the shooter turret.
 */
public class Turret extends SubsystemBase {

  // TODO: min and max values need to be figured out; the values below are fictious values.
  private static final double MIN_ENCODER_VALUE = 0;
  private static final double MAX_ENCODER_VALUE = 1024;

  private final VictorSPX turretMotor = new VictorSPX(TurretConstants.kTurretMotorPort); // Change back to victor when we get the actual robot
  private final Encoder turretEncoder = new Encoder(TurretConstants.kTurretEncoderPorts[0], TurretConstants.kTurretEncoderPorts[1]);
  private PIDController turretPIDController;
  private double maxPower;

  /**
   * Creates the Turret subsystem.
   */
  public Turret() {
  }

  /**
   * Initializes PID controller for turret.
   * 
   * @param desiredAngleX
   * @param tolerance
   */
  public void turretAnglePIDInit(double desiredAngleX, double maxPower, double tolerance) {
    double kP = NRGPreferences.TURRET_P_TERM.getValue();
    double kI = NRGPreferences.TURRET_I_TERM.getValue();
    double kD = NRGPreferences.TURRET_D_TERM.getValue();

    this.turretPIDController = new PIDController(kP, kI, kD);
    this.turretPIDController.setSetpoint(desiredAngleX);
    this.turretPIDController.setTolerance(tolerance);
    
    this.maxPower = maxPower;
  }

  /**
   * Updates turret motor power based on output from the PID controller.
   * 
   * @param limelightAngleX from limelight is used to calculate power
   */
  public void turretAngleToExecute(double limelightAngleX) {
    double currentPower = this.turretPIDController.calculate(limelightAngleX) * maxPower;
    rawTurret(currentPower);
  }

  /**
   * Passes power to turret with hard stop protection.
   * @param power
   */
  public void rawTurret(double power){
    int encoderTicks = turretEncoder.get();
    //Prevent the turret from turning past hard stops
    // if (encoderTicks >= MAX_ENCODER_VALUE && power > 0 || encoderTicks < MIN_ENCODER_VALUE && power < 0){
    //   power = 0;
    // }
    turretMotor.set(ControlMode.PercentOutput, power);
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
    this.turretMotor.set(ControlMode.PercentOutput, 0);
    this.turretPIDController = null;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/Distance", turretEncoder.getDistance());
  }
}
