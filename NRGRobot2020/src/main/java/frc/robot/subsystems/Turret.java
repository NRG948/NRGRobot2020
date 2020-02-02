/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.NRGPreferences;

public class Turret extends SubsystemBase {

  private Victor turretMotor = new Victor(0);
  private Encoder turretEncoder = new Encoder(5, 6);
  private PIDController turretPIDController;
  private double maxPower;
  /**
   * TODO: min and max values need to be figured out; the values below are fictious values.
   */
  private static final double MIN_ENCODER_VALUE = 0;
  private static final double MAX_ENCODER_VALUE = 1024;

  /**
   * Creates a new Turret.
   */
  public Turret() {
  }

  /**
   * Initializes PID controller for turret.
   * 
   * @param desiredAngle
   * @param tolerance
   */
  public void turretAnglePIDInit(double desiredAngle, double maxPower, double tolerance) {
    double kP = NRGPreferences.TURRET_P_TERM.getValue();
    double kI = NRGPreferences.TURRET_I_TERM.getValue();
    double kD = NRGPreferences.TURRET_D_TERM.getValue();

    this.turretPIDController = new PIDController(kP, kI, kD);
    this.turretPIDController.setSetpoint(0);
    this.turretPIDController.setTolerance(tolerance);

    this.maxPower = maxPower;
  }


  /**
   * Updates turret motor power based on output from the PID controller.
   * 
   * @param maxPower is the largest power sent to motor controller.
   */
  public void turretAngleToExecute(double limelightAngleX) {
    double currentPower = this.turretPIDController.calculate(limelightAngleX) * maxPower;
    int encoderTicks = turretEncoder.get();
    if (encoderTicks >= MAX_ENCODER_VALUE && currentPower > 0 || encoderTicks <= MIN_ENCODER_VALUE && currentPower < 0){
      currentPower = 0;
    }
    turretMotor.set(currentPower);
  }

  /**
   * Returns whether the turret is within the tolerance of the setpoint.
   * @return true when turret angle is on target
   */
  public boolean turretAngleOnTarget() {
    return this.turretPIDController.atSetpoint();
  }

  /**
   * Stops the turretMotor
   * 
   */
  public void turretAngleEnd() {
    this.turretMotor.stopMotor();
    this.turretPIDController = null;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
