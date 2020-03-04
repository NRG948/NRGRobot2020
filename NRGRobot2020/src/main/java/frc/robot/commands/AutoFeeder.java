/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.NRGPreferences;

public class AutoFeeder extends CommandBase {
  private BallCounter ballCounter;
  private Feeder feeder;
  /**
   * Creates a new AutoFeeder.
   */
  public AutoFeeder(BallCounter ballCounter, Feeder feeder) {
    this.ballCounter = ballCounter;
    this.feeder = feeder;
   /* BallCounter is shared between this command and AcquireNumberOfBalls therfore we do not add it to
      requirements.*/
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feeder.rawFeeder(NRGPreferences.FEEDER_ACQUIRE_POWER.getValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.rawFeeder(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ballCounter.isBallInShootingPosition();
  }
}
