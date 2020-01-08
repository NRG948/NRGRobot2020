/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /**
   * Creates a new ExampleSubsystem.
   */
  private Victor driveFrontRightMotor = new Victor(0);
  private Victor driveMiddleRightMotor = new Victor(1);
  private Victor driveBackRightMotor = new Victor(2);
  private Victor driveFrontLeftMotor = new Victor(3);
  private Victor driveMiddleLeftMotor = new Victor(4);
  private Victor driveBackLeftMotor = new Victor(5);
  
  public Drive() {

  }
  public void tankDrive(double leftPower, double rightPower){
    driveFrontRightMotor.set(rightPower);
    driveMiddleRightMotor.set(rightPower);
    driveBackRightMotor.set(rightPower);
    driveFrontLeftMotor.set(leftPower);
    driveMiddleLeftMotor.set(leftPower);
    driveBackLeftMotor.set(leftPower);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
