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


public class Acquirer extends SubsystemBase {

  private double sentPower;
  private SimpleWidget acquirerRawOutputWidget;

  private Victor acquirerVictor = new Victor(2);
  /**
   * Creates a new Acquirer.
   */


  public Acquirer() {

  }

  public void rawAcquirer(double power){
    sentPower = power * 0.5;
    acquirerRawOutputWidget.getEntry().setDouble(sentPower);
    acquirerVictor.set(sentPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void initShuffleboard(){
    ShuffleboardTab acquirerTab = Shuffleboard.getTab("Acquirer");

    ShuffleboardLayout acquirerLayout = acquirerTab.getLayout("Acquirer", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 4);
    acquirerLayout.add("Raw Output", 0.0);
  }
}