/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.AddressableLEDs;

public class LEDTest extends CommandBase {

  int counter = 0;
  public LEDTest(AddressableLEDs addressableLEDs) {
    addRequirements(addressableLEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    AddressableLEDs.setAll(new Color8Bit(0,0,0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    AddressableLEDs.set(0, new Color8Bit(counter,0,0));
    AddressableLEDs.set(2, new Color8Bit(0,counter,0));
    AddressableLEDs.set(4, new Color8Bit(0,0,counter));
    AddressableLEDs.set(6, new Color8Bit(counter,counter,0));
    AddressableLEDs.set(7, new Color8Bit(counter,0,counter));
    AddressableLEDs.sendToLeds();
    counter = (counter + 1) % 256;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
