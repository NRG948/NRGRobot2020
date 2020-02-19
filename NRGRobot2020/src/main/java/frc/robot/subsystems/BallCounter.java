/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallCounter extends SubsystemBase {

  private static final boolean BEAM_BROKEN = false;
  private static final boolean BEAM_CONNECTED = true;

  private DigitalInput acquirerBeamBreak = new DigitalInput(9);
  private DigitalInput feederBeamBreak = new DigitalInput(8);
  private boolean acquirerLastState = acquirerBeamBreak.get();
  private boolean feederLastState = feederBeamBreak.get();
  private int ballCount = 3;
  /**
   * Creates a new BallCounter.
   */
  public BallCounter() {

  }

  /**
   * Returns true if a ball is in shooting postition
   */
  public boolean isBallInShootingPosition(){
    return feederBeamBreak.get();
  }

  /**
   * Returns the ball count
   */
  public int getBallCount() {
    return ballCount;
  }

  @Override
  public void periodic() {
    boolean acquirerCurrentState = acquirerBeamBreak.get();
    boolean feederCurrentState = feederBeamBreak.get();
    if (acquirerCurrentState != acquirerLastState && acquirerLastState == BEAM_CONNECTED){
      ++ballCount;
    }
    if (feederCurrentState != feederLastState && feederLastState == BEAM_BROKEN && ballCount > 0){
      --ballCount;
    }
  }
}
