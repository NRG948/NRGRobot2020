/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallTracker;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.MathUtil;
import frc.robot.utilities.NRGPreferences;
import frc.robot.vision.BallTarget;

public class DriveToBall extends CommandBase {
  private final Drive drive;
  private final BallTracker ballTracker;
  private double maxPower;
  private boolean ballNotFound;
  private static final double IMAGE_FOV_DEGREES = 64.4;
  private static final double INCHES_TO_SLOW = 20;
  private int ballNotFoundCounter = 0;

  public DriveToBall(Drive drive, BallTracker ballTracker) {
    this.drive = drive;
    this.ballTracker = ballTracker;
    maxPower = NRGPreferences.NumberPrefs.DRIVE_TO_BALL_MAXPOWER.getValue();
    addRequirements(drive);
  }

  public DriveToBall withMaxPower(double maxPower) {
    this.maxPower = maxPower;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballNotFound = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    BallTarget ballTarget = ballTracker.getBallTarget();
    if (ballTarget != null) {
      ballNotFoundCounter = 0;
      double distanceToTarget = ballTarget.distanceToTarget();
      double angleToTarget = ballTarget.getAngleToTarget();
      double turnPower = angleToTarget / IMAGE_FOV_DEGREES;
      double drivePower = MathUtil.clamp((distanceToTarget / INCHES_TO_SLOW), 0.4, maxPower);
      drive.arcadeDrive(drivePower, turnPower);
      SmartDashboard.putNumber("DriveToBall/Distance", distanceToTarget);
      SmartDashboard.putNumber("DriveToBall/Angle", angleToTarget);
      SmartDashboard.putNumber("DriveToBall/turnPower", turnPower);
      SmartDashboard.putNumber("DriveToBall/drivePower", drivePower);
    } else {
      ballNotFoundCounter++;
      if(ballNotFoundCounter>25){
        ballNotFound = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ballNotFound;
  }
}
