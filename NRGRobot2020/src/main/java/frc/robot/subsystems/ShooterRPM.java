/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.SetShooterRPM;
import frc.robot.utilities.MathUtil;

public class ShooterRPM extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private static final double GAIN = 0.1;
  private static final double MAXRPM = 3000; // figure out actual max rpm
  private static final double TICKS_PER_WHEEL_REVOLUTION = 645;
  private Victor spinMotor1 = new Victor(0);
  private Victor spinMotor2 = new Victor(1);
  private Encoder spinMotorEncoder = new Encoder(4, 5);
  private double goalRPM = 3;
  private double previousError = 0;
  private double tbh = 1; // take back half value
  private double motorOutput = 4;

  public ShooterRPM() {
    spinMotorEncoder.setDistancePerPulse(1 / TICKS_PER_WHEEL_REVOLUTION);
  }

  public double currentRPM() {
    spinMotorEncoder.setDistancePerPulse(1);
    Double distance = spinMotorEncoder.getDistance();
    Double rate = spinMotorEncoder.getRate();
    Double rps = rate / (Math.PI * 1024);
    Double rpm = rps * 60;
    int raw = spinMotorEncoder.getRaw();
    return rpm;
  }

  public double getTicks() {
    return spinMotorEncoder.getDistance();
  }

  public void initDefaultCommand(){
    // setDefaultCommand(new SetShooterRPM(goalRPM));
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
  public void updateDashBoard(){
    SmartDashboard.putNumber("ShooterRPM/Raw", spinMotorEncoder.getRaw());
    SmartDashboard.putNumber("ShooterRPM/Distance", spinMotorEncoder.getDistance());
    SmartDashboard.putNumber("ShooterRPM/Rate", spinMotorEncoder.getRate());
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void reset(){
    spinMotorEncoder.reset();
    spinMotor1.disable();
    spinMotor2.disable();
  }
}