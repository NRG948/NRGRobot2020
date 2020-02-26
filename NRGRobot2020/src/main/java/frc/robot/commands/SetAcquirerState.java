/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AcquirerPiston;
import frc.robot.subsystems.AcquirerPiston.State;

public class SetAcquirerState extends CommandBase {
  private AcquirerPiston acquirerPiston;
  private State state;
  /**
   * Creates a new SetAcquirerState.
   */
  public SetAcquirerState(AcquirerPiston acquirerPiston, State state) {
    this.acquirerPiston = acquirerPiston;
    this.state = state;
    addRequirements(acquirerPiston);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    acquirerPiston.setState(state);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
