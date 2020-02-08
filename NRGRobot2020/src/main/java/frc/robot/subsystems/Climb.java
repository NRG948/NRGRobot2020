/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Climb extends SubsystemBase {
  /**
   * Creates a new climb.
   */
  public Climb() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void initShuffleboard(){
    ShuffleboardTab climbTab = Shuffleboard.getTab("Climb");

    ShuffleboardLayout climbLayout = climbTab.getLayout("Climb", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
  }
}
