/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterRPM;

public class SetShooterRPM extends CommandBase {
  private final double goalRPM;
  private final ShooterRPM shooterRPM;
  private Timer timer = new Timer();
  /**
   * Creates a new SetShooterRPM.
   */
  public SetShooterRPM(double goalRPM, ShooterRPM shooterRPM) {
    this.goalRPM = goalRPM;
    this.shooterRPM = shooterRPM;
    addRequirements(shooterRPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooterRPM.setGoalRPM(goalRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterRPM.updateRPM();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get()>5){
      timer.stop();
      return true;
    }
    return false;
  }
}
