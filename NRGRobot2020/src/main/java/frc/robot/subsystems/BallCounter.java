/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BallCounter extends SubsystemBase {

  private static final boolean BEAM_BROKEN = false;
  private static final boolean BEAM_CONNECTED = true;

  private DigitalInput acquirerBeamBreak = new DigitalInput(9);
  private DigitalInput feederBeamBreak = new DigitalInput(8);
  private boolean acquirerLastState = acquirerBeamBreak.get();
  private boolean feederLastState = feederBeamBreak.get();
  private int ballCount = 3;
  private SimpleWidget ballCountWidget;

  /**
   * Creates a new BallCounter.
   */
  public BallCounter() {

  }

  /**
   * Returns true if a ball is in shooting postition
   */
  public boolean isBallInShootingPosition() {
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
    if (acquirerCurrentState != acquirerLastState && acquirerLastState == BEAM_CONNECTED) {
      ++ballCount;
      updateBallCountWidget();
    }
    if (feederCurrentState != feederLastState && feederLastState == BEAM_BROKEN && ballCount > 0) {
      --ballCount;
      updateBallCountWidget();
    }
  }

  public void addShuffleboardTab() {
    ShuffleboardTab ballCounterTab = Shuffleboard.getTab("Ball Counter");
    ShuffleboardLayout layout = ballCounterTab.getLayout("Ball Counter", BuiltInLayouts.kList).withPosition(0, 0)
        .withSize(2, 3);
    layout.addBoolean("Acquirer Beam Break", () -> acquirerBeamBreak.get());
    layout.addBoolean("Feeder  Beam Break", () -> feederBeamBreak.get());
    ballCountWidget = layout.add("Ball Count", ballCount);
  }

  private void updateBallCountWidget() {
    if (ballCountWidget != null) {
      ballCountWidget.getEntry().forceSetNumber(ballCount);
    }
  }
}
