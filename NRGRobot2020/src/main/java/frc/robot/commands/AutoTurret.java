/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.Turret;

public class AutoTurret extends CommandBase {
  private final Turret turret;
  private double maxPower;
  private final LimelightVision limelight;
  // desiredAngle is relative?
  private double desiredAngle;

  /**
   * Creates a new AutoTurret.
   */
  public AutoTurret(final Turret turret, final LimelightVision limelight) {
    this.turret = turret;
    this.limelight = limelight;

    addRequirements(turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public AutoTurret withMaxPower(double maxPower) {
    this.maxPower = maxPower;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    turret.turretAnglePIDInit(0, maxPower, 2.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    desiredAngle = limelight.getX();
    turret.turretAngleToExecute(desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.turretAngleEnd();
  }

  // Returns true when the command should end.
  /**
   * isFinished can be used to detect if a switch is toggled or button is pressed.
   * Or, it can be not used, and this would be a continous command.
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}
