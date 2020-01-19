/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class SimplePIDController extends PIDSubsystem {

  public static final double BOTTOM = 4.6;
  public static final double STOW = 1.65;
  public static final double TABLE_HEIGHT = 1.58;
  /**
   * Creates a new Elevator.
   */
  public SimplePIDController() {
    super(new PIDController(0, 0, 0)); // The PIDController used by the subsystem

      setSetpoint(STOW);
      enable();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  protected double returnPIDInput() {
    return 0;
 }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
