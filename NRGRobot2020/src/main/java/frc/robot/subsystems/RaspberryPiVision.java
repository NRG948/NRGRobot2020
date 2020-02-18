package frc.robot.subsystems;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetRaspberryPiPipeline;
import frc.robot.vision.FuelCellTarget;
import frc.robot.vision.LoadingStationTarget;
import frc.robot.vision.VisionTarget;
import frc.robot.Constants.RaspberryPiConstants;

import org.opencv.core.Mat;

public class RaspberryPiVision extends SubsystemBase {
  /**
   * Enumeration representing pipeline to run
   */
  public enum PipelineRunner {
    FUEL_CELL("FuelCellTrackingRunner", RaspberryPiConstants.kWhite),
    LOADING_STATION("LoadingStationRunner", RaspberryPiConstants.kGreen);

    private final String name;
    private final Color8Bit color;

    /**
     * constructs the enumeration
     * 
     * @param name  name of the pipeline
     * @param color color of the leds
     */
    PipelineRunner(String name, Color8Bit color) {
      this.name = name;
      this.color = color;
    }

    /**
     * 
     * @return returns the name of the pipeline
     */
    public String getName() {
      return this.name;
    }

    /**
     * 
     * @return returns the color of the led
     */
    public Color8Bit getColor() {
      return this.color;
    }
  }

  private VisionTarget currentTarget;
  private FuelCellTarget fuelCellTarget;
  private LoadingStationTarget loadingStationTarget;
  private PipelineRunner currentRunner;

  /**
   * Creates a new RaspberryPiVision.
   */
  public RaspberryPiVision() {
    setPipelineRunner(PipelineRunner.FUEL_CELL);
  }

  /**
   * sets the pipeline to run on the rasberry pi
   * 
   * @param runner value from the pipeline enumeration
   */
  public void setPipelineRunner(PipelineRunner runner) {
    SmartDashboard.putString("Vision/runnerName", runner.getName());
    AddressableLEDs.setAll(runner.getColor());
    AddressableLEDs.sendToLeds();
    currentRunner = runner;
  }

  /**
   * returns the vision data generation count
   */
  public int getGenCount() {
    return (int) SmartDashboard.getNumber("Vision/genCount", 0);
  }

  /**
   * returns the current vision target
   * 
   * @return the current vision target
   */
  public VisionTarget getCurrentTarget() {
    return this.currentTarget;
  }

  /**
   * Gets the current fuel cell target.
   * 
   * @return The fuel cell target, or null if none.
   */
  public FuelCellTarget getFuelCellTarget() {
    return this.fuelCellTarget;
  }

  public LoadingStationTarget getLoadingTarget() {
    return this.loadingStationTarget;
  }

  private void updateFuelCell() {
    boolean hasTarget = SmartDashboard.getBoolean("Vision/fuelCell/hasTarget", false);
    if (hasTarget) {
      double distance = SmartDashboard.getNumber("Vision/fuelCell/distance", 0);
      double angle = SmartDashboard.getNumber("Vision/fuelCell/angle", 0);
      this.fuelCellTarget = new FuelCellTarget(distance, angle);
    } else {
      this.fuelCellTarget = null;
    }
  }

  private void updateLoadingStation() {
    boolean hasTarget = SmartDashboard.getBoolean("Vision/LoadingStation/HasTarget", false);
    if (hasTarget) {
      double distance = SmartDashboard.getNumber("Vision/LoadingStation/DistanceInches", 0.0);
      double angle = SmartDashboard.getNumber("Vision/LoadingStation/Angle", 0.0);
      double skew = SmartDashboard.getNumber("Vision/LoadingStation/Skew", 0.0);
      this.loadingStationTarget = new LoadingStationTarget(distance, angle, skew);
    } else {
      this.loadingStationTarget = null;
    }
  }

  private void updateCurrentTarget() {
    updateLoadingStation();
    updateFuelCell();
    if (loadingStationTarget != null) {
      currentTarget = loadingStationTarget;
    } else if (fuelCellTarget != null) {
      currentTarget = fuelCellTarget;
    } else {
      currentTarget = null;
    }
  }

  /**
   * Adds a Shuffleboard tab for the Raspberry Pi subsystem.
   */
  public void addShuffleBoardTab() {
    ShuffleboardTab piTab = Shuffleboard.getTab("RaspberryPi");

    // Create a list layout and add the buttons to change the pipeline.
    ShuffleboardLayout pipelineLayout = piTab.getLayout("Pipeline", BuiltInLayouts.kList).withPosition(0, 0).withSize(2,
        2);

    pipelineLayout.addString("Pipeline Runner", () -> currentRunner.getName());
    pipelineLayout.add("LoadingBayTarget", new SetRaspberryPiPipeline(this, PipelineRunner.LOADING_STATION));
    pipelineLayout.add("FuelCellTrackerTarget", new SetRaspberryPiPipeline(this, PipelineRunner.FUEL_CELL));

    // Adds the processed video to the RaspberryPi Shuffleboard tab.
    VideoSource processedVideo = new HttpCamera("Processed", "http://frcvision.local:1181/stream.mjpg");

    piTab.add("Processed Video", processedVideo).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0).withSize(4,
        3);

    // Creating a list layout and add current vision target information
    ShuffleboardLayout targetLayout = piTab.getLayout("Current Target", BuiltInLayouts.kList).withPosition(6, 0)
        .withSize(2, 3);
    targetLayout.addBoolean("Has target", () -> this.getCurrentTarget() != null).withWidget(BuiltInWidgets.kBooleanBox);
    targetLayout.addNumber("Distance",
        () -> this.currentTarget != null ? this.currentTarget.getDistanceInInches() : 0.0);
    targetLayout.addNumber("Angle", () -> this.currentTarget != null ? this.currentTarget.getAngleInDegrees() : 0.0);
    targetLayout.addNumber("Skew", () -> this.currentTarget != null ? this.currentTarget.getSkewInDegrees() : 0.0);
    // When running in the simulator, we'll use the default USB camera to create the
    // "Processed" video server.
    if (!RobotBase.isReal()) {
      simulateProcessedVideo();
    }
  }

  /**
   * Simulates the processed video stream when running on the robot simulator.
   */
  private void simulateProcessedVideo() {
    var cameraThread = new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

      CvSink cameraSink = CameraServer.getInstance().getVideo();
      CvSource processedSource = CameraServer.getInstance().putVideo("Processed", camera.getVideoMode().width,
          camera.getVideoMode().height);

      Mat source = new Mat();

      while (!Thread.interrupted()) {
        if (cameraSink.grabFrame(source) == 0) {
          continue;
        }
        processedSource.putFrame(source);
      }
    });

    cameraThread.setDaemon(true);
    cameraThread.start();
  }

  @Override
  public void periodic() {
    updateCurrentTarget();
  }
}
