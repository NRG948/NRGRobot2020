/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TurnToHeading extends CommandBase {
  
  private final double DEFAULT_TURN_TOLERANCE = 5.0;

  private double desiredHeading = 0; // the heading we want the robot to end at
  private double maxPower = 0; // gives the maximum power the robot is gonna drive when the command is executed
  private Drive drive;
/**
   * Creates a new TurnToHeading.
   */
  public TurnToHeading(Drive drive) {
    this.drive = drive;
    addRequirements(drive); // requires the Drive subsystem
  }

  /**
   * Sets the max power to turn the robot
   * @param maxPower the max power
   * @return returns this
   */
  public TurnToHeading withMaxPower(double maxPower){
    this.maxPower = maxPower;
    return this;
  }

/**
 * sets the heading to desired heading
 * @param heading the heading
 * @return returns this
 */
  public TurnToHeading toHeading(double heading){
    this.desiredHeading = heading;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // this gives in the angle into the command and intializes the command and gives
    // in the tolerance
    drive.turnToHeadingInit(this.desiredHeading, DEFAULT_TURN_TOLERANCE);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // executes the turn at the maximum speed
      drive.turnToHeadingExecute(this.maxPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.turnToHeadingEnd(); // terminates the turn
  }

  // Checks whether the robot is on target 
  @Override
  public boolean isFinished() {
    return drive.turnToHeadingOnTarget(); 
}
}
