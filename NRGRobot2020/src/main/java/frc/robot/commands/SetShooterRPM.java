/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterRPM;

public class SetShooterRPM extends CommandBase {
  private final double RPM;
  private final ShooterRPM shooterRPM;
  /**
   * Creates a new SetShooterRPM.
   */
  public SetShooterRPM(double RPM, ShooterRPM shooterRPM) {
    this.RPM = RPM;
    this.shooterRPM = shooterRPM;
    // addRequirements(new RobotContainer().shooterRPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterRPM.setGoalRPM(RPM);
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
