/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.Logger;

/**
 * Sets the initial robot odometry to a given pose (an x,y position and robot heading).
 * 
 * Should be invoked by each autonomous routine option based on the robot's starting position.
 */
public class SetStartPosition extends CommandBase {
  private Drive drive;
  private Pose2d pose;
  
  /**
   * Creates a new SetStartPosition command.
   */
  public SetStartPosition(Drive drive, Pose2d pose) {
    this.drive = drive;
    this.pose = pose;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Logger.commandInit(this, "pose: " + pose);
    drive.resetOdometry(pose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Logger.commandEnd(this, interrupted);
  }

  // Returns true when the command should end, which is immediately.
  @Override
  public boolean isFinished() {
    return true;
  }
}
