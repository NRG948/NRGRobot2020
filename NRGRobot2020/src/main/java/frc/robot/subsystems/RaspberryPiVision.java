package frc.robot.subsystems;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.RaspberryPiPipelines;
import frc.robot.vision.FuelCellTarget;
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
    update();
    return !ballTargets.isEmpty() ? ballTargets.get(0) : null;
  }

  public void update() {
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

  public void addShuffleBoardTab() {
    ShuffleboardTab piTab = Shuffleboard.getTab("RaspberryPi");
    piTab.add("LoadingBayTarget", new RaspberryPiPipelines(this, PipelineRunner.LOADING_STATION));
    piTab.add("FuelCellTrackerTarget", new RaspberryPiPipelines(this, PipelineRunner.FUEL_CELL));
  }

  @Override
  public void periodic() {
  }
}
