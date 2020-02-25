package frc.robot.subsystems;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SetRaspberryPiPipeline;
import frc.robot.utilities.NRGPreferences;
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
    /**
     * The pipeline runner for tracking fuel cells.
     */
    FUEL_CELL("FuelCellTrackingRunner", RaspberryPiConstants.kWhite),

    /**
     * The pipeline runner for locating the loading station.
     */
    LOADING_STATION("LoadingStationRunner", RaspberryPiConstants.kGreen);

    private final String name;
    private final Color8Bit color;

    /**
     * Constructs the enumeration.
     * 
     * @param name  The name of the pipeline.
     * @param color The color of the LEDs.
     */
    PipelineRunner(String name, Color8Bit color) {
      this.name = name;
      this.color = color;
    }

    /**
     * Returns the name of the pipeline.
     * 
     * @return The name of the pipeline.
     */
    public String getName() {
      return this.name;
    }

    /**
     * Returns the color of the LEDs.
     * 
     * @return The color of the LEDs.
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
   * Sets the pipeline to run on the Raspberry Pi.
   * 
   * @param runner A value from the pipeline enumeration representing the pipeline
   *               to run.
   */
  public void setPipelineRunner(PipelineRunner runner) {
    SmartDashboard.putString("Vision/runnerName", runner.getName());
    AddressableLEDs.setAll(runner.getColor());
    AddressableLEDs.sendToLeds();
    currentRunner = runner;
  }

  /**
   * Returns the vision data generation count.
   * 
   * @return The vision data generation count.
   */
  public int getGenCount() {
    return (int) SmartDashboard.getNumber("Vision/genCount", 0);
  }

  /**
   * Returns the current vision target.
   * 
   * @return the current vision target, or null if none.
   */
  public VisionTarget getCurrentTarget() {
    return this.currentTarget;
  }

  /**
   * Returns the current fuel cell target.
   * 
   * @return The fuel cell target, or null if none.
   */
  public FuelCellTarget getFuelCellTarget() {
    return this.fuelCellTarget;
  }

  /**
   * Returns the current loading station target.
   * 
   * @return The current loading station target, or null if none.
   */
  public LoadingStationTarget getLoadingTarget() {
    return this.loadingStationTarget;
  }

  /**
   * Called from the periodic method to update the fuel cell target information
   * from the vision data.
   */
  private void updateFuelCellTarget() {
    boolean hasTarget = SmartDashboard.getBoolean("Vision/fuelCell/hasTarget", false);

    if (hasTarget) {
      double distance = SmartDashboard.getNumber("Vision/fuelCell/distance", 0);
      double angle = SmartDashboard.getNumber("Vision/fuelCell/angle", 0);
      this.fuelCellTarget = new FuelCellTarget(distance, angle);
    } else {
      this.fuelCellTarget = null;
    }
  }

  /**
   * Called from the periodic method to update the loading station target
   * information from the vision data.
   */
  private void updateLoadingStationTarget() {
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

  /**
   * Called from the periodic method to update the current target information from
   * the vision data.
   */
  private void updateCurrentTarget() {
    updateLoadingStationTarget();
    updateFuelCellTarget();
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
    if (!NRGPreferences.SHUFFLEBOARD_RASPBERRY_PI_ENABLED.getValue()){
      return;
    }
    
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

  /**
   * Called by the scheduler to perform periodic tasks before commands exectue.
   */
  @Override
  public void periodic() {
    updateCurrentTarget();
  }
}
