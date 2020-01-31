/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.vision.BallTarget;
import java.util.*;

public class BallTracker extends SubsystemBase {
  private static final String[] NO_BALL_TARGETS = new String[0];
  private Gson gson = new Gson();
  private ArrayList<BallTarget> ballTargets = new ArrayList<BallTarget>();

  /**
   * Creates a new BallTracker.
   */
  public BallTracker() {

  }

  public BallTarget getBallTarget(){
    update();
    return !ballTargets.isEmpty()?ballTargets.get(0):null;
  }

  public void update() {
    // This method will be called once per scheduler run
    String[] ballTargetsJson = SmartDashboard.getStringArray("Vision/ballTargets", NO_BALL_TARGETS);
    ArrayList<BallTarget> tempBallTargets = new ArrayList<BallTarget>();
    for (String ballTargetJson : ballTargetsJson) {
      tempBallTargets.add(gson.fromJson(ballTargetJson, BallTarget.class));
    }

    ballTargets = tempBallTargets;

    boolean hasTargets = !ballTargets.isEmpty();
    SmartDashboard.putBoolean("Vision/hasTargets", hasTargets);
    if (hasTargets) {
      SmartDashboard.putNumber("Vision/distanceToTarget", ballTargets.get(0).distanceToTarget());
      SmartDashboard.putNumber("Vision/angleToTarget", ballTargets.get(0).getAngleToTarget());
    }
  }

  @Override
  public void periodic() {
  }
}
