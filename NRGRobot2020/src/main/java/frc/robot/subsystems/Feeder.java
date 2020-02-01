/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private Victor feederVictor = new Victor(3);

  /**
   * Creates a new Feeder.
   */
  public Feeder() {

  }

  public void rawFeeder(double power){
    feederVictor.set(power * 0.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
