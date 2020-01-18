/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
 
package frc.robot.subsystems;
 
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.MathUtil;
 
public class ShooterRPM extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private static final double GAIN = 0.1;
  private static final double MAXRPM = 3000; // figure out actual max rpm
  private static final double TICKS_PER_WHEEL_REVOLUTION = 1;
  private Victor spinMotor1 = new Victor(0);
  private Victor spinMotor2 = new Victor(1);
  private Encoder spinMotorEncoder = new Encoder(0, 0);
  private double goalRPM = 0;
  private double previousError = 0;
  private double tbh = 0; // take back half value
  private double motorOutput = 0;
 
  public ShooterRPM() {
    spinMotorEncoder.setDistancePerPulse(1/TICKS_PER_WHEEL_REVOLUTION);
  }
 
  public double currentRPM() {
    return spinMotorEncoder.getRate(); //
  }
 
  public double getTicks() {
    return spinMotorEncoder.getDistance();
  }
 
  public void updateRPM() {
    double error = goalRPM - currentRPM(); // calculate the error;
    motorOutput += GAIN * error; // integrate the output;
    motorOutput = MathUtil.clamp(motorOutput, 0, 1);
    if (error * previousError < 0) { // if it crossed the goal RPM
      motorOutput = 0.5 * (motorOutput + tbh); // then Take Back Half
      tbh = motorOutput; // update Take Back Half variable
      previousError = error; // and save the previous error
    }
    setFlyWheel(motorOutput);
  }
 
  public void setGoalRPM(double goalRPM) {
    this.goalRPM = goalRPM;
    if (goalRPM == 0) {
      motorOutput = 0;
      previousError = 0;
      tbh = 0;
    } else {
      motorOutput = 1;
      previousError = 1;
      tbh = 2 * guessMotorOutputFromRPM(goalRPM) - 1;
    }
 
  }
 
  private double guessMotorOutputFromRPM(double RPM) {
    return MathUtil.clamp(RPM / MAXRPM, 0, 1);
  }
 
  public void setFlyWheel(double power) {
    spinMotor1.set(power);
    spinMotor2.set(power);
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}