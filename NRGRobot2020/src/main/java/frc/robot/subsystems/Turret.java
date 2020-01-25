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
  public void turretAnglePIDInit(double desiredAngle, double tolerance) {
    double p = NRGPreferences.NumberPrefs.TURRET_P_TERM.getValue();
    double i = NRGPreferences.NumberPrefs.TURRET_I_TERM.getValue();
    double d = NRGPreferences.NumberPrefs.TURRET_D_TERM.getValue();

    this.turretPIDController = new PIDController(p, i, d);
    this.turretPIDController.setSetpoint(desiredAngle);
    this.turretPIDController.setTolerance(tolerance);
  }

  private double encoderToAngle() {
    // TODO: Convert ticks to angle
    return turretEncoder.get();
  }

  /**
   * Updates turret motor power based on output from the PID controller.
   * 
   * @param maxPower is the largest power sent to motor controller.
   */
  public void turretAngleToExecute(double maxPower) {
    double currentPower = this.turretPIDController.calculate(encoderToAngle()) * maxPower;
    turretMotor.set(currentPower);
  }

  /**
   * 
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
