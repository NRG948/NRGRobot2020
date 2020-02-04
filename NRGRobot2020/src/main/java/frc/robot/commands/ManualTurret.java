/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ManualTurret extends CommandBase {
  /**
   * Creates a new ManualTurret.
   */
  final XboxController m_xboxController;
  final Turret m_turret;
  private double power;

  ; 

  public ManualTurret(Turret turret, XboxController xboxController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_xboxController = xboxController;
    this.m_turret = turret;
    addRequirements(m_turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    power = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_xboxController.getPOV() == 90){
      power += 0.05;
    }
    else if(m_xboxController.getPOV() == 270){
      power -= 0.05;
    }
    m_turret.rawTurret(power);
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
