/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static java.lang.System.out;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

import frc.robot.subsystems.Drive;

public class DriveStraightDistance extends CommandBase {
  private final Drive drive;
  private double xOrigin;// a class variable
  private double yOrigin;
  private final double distance;// this is a constant
  private final double maxPower;
  private final boolean stopMotors;

  private boolean useRobotHeading = true;
  private double heading = 0;
  /**
   * Creates a new DriveStraightDistance.
   */
  public DriveStraightDistance(final Drive subsystem, double distance, double maxPower, boolean stopMotors) {
    this.drive = subsystem;
    this.distance = distance; // sets the value of the distance that we want to travel
    this.maxPower = maxPower; // sets the maximum power
    this.stopMotors = stopMotors;
  }

  public DriveStraightDistance(final Drive subsystem, double distance, double maxPower) {
    this(subsystem, distance, maxPower, true);
  }
      
  public DriveStraightDistance(final Drive subsystem, double heading, double distance, double maxPower, boolean stopMotors) {
    this(subsystem, distance, maxPower, stopMotors);
    this.useRobotHeading = false;
    this.heading = heading;
  }

  // Called when the command
  @Override
  public void initialize() { 
    if (useRobotHeading) {
      this.heading = Robot.navx.getAngle();
    }
    System.out.println("DriveStraightDistance Init heading: " + heading + " distance: " + distance);
    this.xOrigin = this.drive.getPose().getTranslation().getX(); // gets our current X position from the poistion tracker command
    this.yOrigin = this.drive.getPose().getTranslation().getY(); // gets our current Y position from the poistion tracker command
    this.drive.driveOnHeadingInit(this.heading); // We are getting our current heading and putting it into
                                            // driveOnHeadingInit to adjust our current heading

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {  
    this.drive.driveOnHeadingExecute(this.maxPower); // excuting the command and puts in the maximum power that the
    // robot is gonna run on  
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (stopMotors) {
      this.drive.driveOnHeadingEnd();
    }
    System.out.println(String.format("DriveStraightDistance End x:%.1f y:%.1f", this.drive.getPose().getTranslation().getX(),
        this.drive.getPose().getTranslation().getY()));
    // terminated the command as the robot has reached the distance that needs to be
    // traveled or if it needs to be interrupted
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.drive.calculateDistance(this.xOrigin, this.yOrigin) >= this.distance;
  }// this calculates the distance that we have traveled from origin in order to
   // figure out if the command
   // needs to be terminated or not and returns true or false true being it needs
   // to be terminated and false being it needs to be continued
}
