/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RaspberryPiPipelines;
import frc.robot.vision.FuelCellTarget;
import frc.robot.vision.LoadingStationTarget;

import java.util.*;

public class RaspberryPiVision extends SubsystemBase {
  public enum PipelineRunner {
    FUEL_CELL("FuelCellTrackingRunner"), LOADING_STATION("LoadingStationRunner");

    private final String name;

    PipelineRunner(String name) {
      this.name = name;
    }

    public String getName() {
      return this.name;
    }
  }

  private static final String[] NO_BALL_TARGETS = new String[0];
  private Gson gson = new Gson();
  private ArrayList<FuelCellTarget> ballTargets = new ArrayList<FuelCellTarget>();

  private double distance;
  private double offsetX;
  private double skew;

  /**
   * Creates a new RaspberryPiVision.
   */
  public RaspberryPiVision() {
    setPipelineRunner(PipelineRunner.FUEL_CELL);
  }

  public void setPipelineRunner(PipelineRunner runner) {
    SmartDashboard.putString("Vision/runnerName", runner.getName());
  }

  public FuelCellTarget getBallTarget() {
    updateFuelCell();
    return !ballTargets.isEmpty() ? ballTargets.get(0) : null;
  }

  public LoadingStationTarget getLoadingTarget() {
    updateLoadingStation();
    return SmartDashboard.getNumber("Vision/LoadingStationCount", 0.0) > 0 ? new LoadingStationTarget(this) : null;
  }

  public void updateFuelCell() {
    String[] ballTargetsJson = SmartDashboard.getStringArray("Vision/ballTargets", NO_BALL_TARGETS);
    ArrayList<FuelCellTarget> tempBallTargets = new ArrayList<FuelCellTarget>();

    for (String ballTargetJson : ballTargetsJson) {
      tempBallTargets.add(gson.fromJson(ballTargetJson, FuelCellTarget.class));
    }

    ballTargets = tempBallTargets;
    boolean hasTargets = !ballTargets.isEmpty();
    SmartDashboard.putBoolean("Vision/hasTargets", hasTargets);

    if (hasTargets) {
      SmartDashboard.putNumber("Vision/distanceToTarget", ballTargets.get(0).distanceToTarget());
      SmartDashboard.putNumber("Vision/angleToTarget", ballTargets.get(0).getAngleToTarget());
    }
  }

  public void updateLoadingStation() {
    distance = SmartDashboard.getNumber("Vision/LoadingStation/DistanceInches", 0.0);
    offsetX =  SmartDashboard.getNumber("Vision/LoadingStation/OffsetX", 0.0);
    skew =  SmartDashboard.getNumber("Vision/LoadingStation/Skew", 0.0);
  }

  /**
   * 
   * @return skew from -1.0 to 1.0 (not linear); taking the inverse cosine of skew returns angle
   */
  public double getLoadingSkew() {
    return skew;
  }

  /**
   * 
   * @return offsetX: the x-distance, from -1.0 to 1.0, from center of robot vision to center of target
   */
  public double getLoadingOffsetX() {
    return offsetX;
  }
  /**
   * 
   * @return distance from robot to Loading Station Target in inches
   */
  public double getLoadingDistance() {
    return distance;
  }

  public void addShuffleBoardTab() {
    ShuffleboardTab piTab = Shuffleboard.getTab("RaspberryPi");
    piTab.add("LoadingBayTarget", new RaspberryPiPipelines(this, PipelineRunner.LOADING_STATION));
    piTab.add("FuelCellTrackerTarget", new RaspberryPiPipelines(this, PipelineRunner.FUEL_CELL));
  }

  @Override
  public void periodic() {
  }
}
