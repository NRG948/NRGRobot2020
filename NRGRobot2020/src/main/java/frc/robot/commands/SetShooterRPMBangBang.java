/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterRPM;
import edu.wpi.first.wpilibj.Timer;

public class SetShooterRPMBangBang extends CommandBase {
  private ShooterRPM shooterRPM;
  private double goalRPM;
  private Timer timer;
  /**
   * Creates a new SetShooterRPMBangBang.
   */
  public SetShooterRPMBangBang() {
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public SetShooterRPMBangBang(double goalRPM,ShooterRPM shooterRPM) {
    this.shooterRPM = shooterRPM;
    this.goalRPM = goalRPM;
    addRequirements(shooterRPM);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.start();
    shooterRPM.setGoalRPM(goalRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterRPM.updateRPMBangBang();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get()>5){
      return true;
    }
    return false;
  }
}
