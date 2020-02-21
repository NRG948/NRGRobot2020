/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.BallCounter;
import frc.robot.utilities.NRGPreferences;

public class AcquireNumberOfBalls extends CommandBase {
  private Acquirer acquirer;
  private BallCounter ballCounter;
  private Timer timer;
  private double acquirerPower;
  private int numberOfBalls;
  private int targetcount;

  /**
   * Creates a new AcquirerForSeconds.
   */
  public AcquireNumberOfBalls(Acquirer acquirer, BallCounter ballCounter, int numberOfBalls) {
    this.acquirer = acquirer;
    this.numberOfBalls = numberOfBalls;
    addRequirements(acquirer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    acquirerPower = NRGPreferences.ACQUIRER_POWER.getValue();
    targetcount = ballCounter.getBallCount() + numberOfBalls;
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
    return (targetcount >= ballCounter.getBallCount());
  }
}
