/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;


public class Hood extends SubsystemBase {
  private Victor hoodVictor = new Victor(3);
  private double sentPower;

  private SimpleWidget rawHoodOutputWidget;
  /**
   * Creates a new Hood.
   */
  public Hood() {

  }
  //TODO: Add encoders for hard limit
  public void rawHood(double power){
    sentPower = power * 0.5;
    rawHoodOutputWidget.getEntry().setDouble(sentPower);
    hoodVictor.set(sentPower);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void initShuffleboard(){
    ShuffleboardTab hoodTab = Shuffleboard.getTab("Hood");

    ShuffleboardLayout hoodLayout = hoodTab.getLayout("Hood", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    rawHoodOutputWidget = hoodLayout.add("Raw Output", 0.0);
  }
}
