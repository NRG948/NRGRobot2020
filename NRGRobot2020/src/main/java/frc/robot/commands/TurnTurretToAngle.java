/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.utilities.Logger;
import frc.robot.utilities.NRGPreferences;
import frc.robot.utilities.TargetSource;

public class TurnTurretToAngle extends CommandBase {
  private Turret turret;
  private double angle;
  private double maxPower;

  /**
   * Creates a new TurnTurretToAngle.
   */
  public TurnTurretToAngle(Turret turret, double angle) {
    this.turret = turret;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    maxPower = NRGPreferences.TURRET_MOTOR_POWER.getValue();
    Logger.commandInit(this, "maxPower: " + maxPower);
    turret.turretAnglePIDInit(TargetSource.NONE, angle, maxPower, 1, false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.turretAngleToExecute(turret.getHeading());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.turretAngleEnd();
    Logger.commandEnd(this, interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turret.turretAngleOnTarget();
  }
}
