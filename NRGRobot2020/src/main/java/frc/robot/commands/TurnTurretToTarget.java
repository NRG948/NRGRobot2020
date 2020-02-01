/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Turret;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnTurretToTarget extends CommandBase {
  private LimelightVision limeLightVision;
  private Turret turret;
  /**
   * Creates a new SetShooterRPM.
   */
  public TurnTurretToTarget(LimelightVision vision, Turret turret) {
    this.limeLightVision = vision;
    this.turret = turret;
  }

  // Someone please help with 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.turretAnglePIDInit(limeLightVision.getX(), 0, 2);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.turretAngleToExecute(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.turretAngleEnd();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.turretAngleOnTarget();
  
  }
}
