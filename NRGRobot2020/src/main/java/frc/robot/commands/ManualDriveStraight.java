/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ManualDriveStraight extends CommandBase {
  private final Drive drive;
  private final Joystick m_leftJoystick;

  /**
   * Creates a new DriveStraight.
   */
  public ManualDriveStraight(final Drive drive,  final Joystick leftJoystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.m_leftJoystick = leftJoystick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double heading = drive.getHeading();
    drive.driveOnHeadingInit(heading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.driveOnHeadingExecute(-m_leftJoystick.getY());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.driveOnHeadingEnd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
