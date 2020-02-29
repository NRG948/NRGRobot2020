/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.BallCounter;
import frc.robot.utilities.NRGPreferences;

public class AcquireNumberOfBalls extends CommandBase {
  private Acquirer acquirer;
  private BallCounter ballCounter;
  private double acquirerPower;
  private int numberOfBalls;
  private int targetcount;
  private boolean absoluteCount;

  /**
   * Creates a new AcquirerForSeconds.
   */
  public AcquireNumberOfBalls(Acquirer acquirer, BallCounter ballCounter) {
    this.acquirer = acquirer;
    this.ballCounter = ballCounter;
    /* BallCounter is shared between this command and AutoFeeder therfore we do not add it to
       requirements.*/
    addRequirements(acquirer);
  }

  public AcquireNumberOfBalls withAbsoluteCount(int numberOfBalls) {
    this.absoluteCount = true;
    this.numberOfBalls = numberOfBalls;
    return this;
  }

  public AcquireNumberOfBalls withRelativeCount(int numberOfBalls) {
    this.absoluteCount = false;
    this.numberOfBalls = numberOfBalls;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    acquirerPower = NRGPreferences.ACQUIRER_POWER.getValue();
    targetcount = numberOfBalls;
    if (!absoluteCount) {
      targetcount += ballCounter.getBallCount();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    acquirer.rawAcquirer(acquirerPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    acquirer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (targetcount <= ballCounter.getBallCount());
  }
}
