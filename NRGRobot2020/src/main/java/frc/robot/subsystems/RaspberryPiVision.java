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
import frc.robot.commands.RaspberryPiPipelines;
import frc.robot.vision.FuelCellTarget;
import frc.robot.vision.LoadingStationTarget;
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

  private FuelCellTarget fuelCellTarget;
  private LoadingStationTarget loadingStationTarget;
  private PipelineRunner currentRunner;

  private final AddressableLED led = new AddressableLED(8);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(12);

  /**
   * Creates a new RaspberryPiVision.
   */
  public RaspberryPiVision() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    setPipelineRunner(PipelineRunner.FUEL_CELL);
  }

  /**
   * sets the pipeline to run on the rasberry pi
   * 
   * @param runner value from the pipeline enumeration
   */
  public void setPipelineRunner(PipelineRunner runner) {
    SmartDashboard.putString("Vision/runnerName", runner.getName());
    for (int i = 0; i < ledBuffer.getLength(); ++i) {
      ledBuffer.setLED(i, runner.getColor());
    }
    led.setData(ledBuffer);
    led.start();
    currentRunner = runner;
  }

  /**
   * Gets the current fuel cell target.
   * 
   * @return The fuel cell target, or null if none.
   */
  public FuelCellTarget getFuelCellTarget() {
    updateFuelCell();
    return this.fuelCellTarget;
  }

  public LoadingStationTarget getLoadingTarget() {
    updateLoadingStation();
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

  /**
   * @return the change in inches for x, y value from starting odometry value to
   *         final point.
   */
  public Translation2d getFinalPoint() {
    double distance = Units.inchesToMeters(this.loadingStationTarget.getDistance());
    double angle = this.loadingStationTarget.getAngleToTarget();
    return new Translation2d(distance * Math.cos(angle), distance * Math.sin(angle));
  }

  /**
   * @return the change in inches for x, y value from starting odometry value to
   *         the waypoint.
   */
  public Translation2d getWaypoint() {
    double distance = Units.inchesToMeters(this.loadingStationTarget.getDistance());
    double angle = this.loadingStationTarget.getAngleToTarget();
    return new Translation2d((distance * Math.cos(angle)) - Units.inchesToMeters(12), distance * Math.sin(angle));
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
    pipelineLayout.add("LoadingBayTarget", new RaspberryPiPipelines(this, PipelineRunner.LOADING_STATION));
    pipelineLayout.add("FuelCellTrackerTarget", new RaspberryPiPipelines(this, PipelineRunner.FUEL_CELL));

    // Adds the processed video to the RaspberryPi Shuffleboard tab.
    VideoSource processedVideo = new HttpCamera("Processed", "http://frcvision.local:1181/stream.mjpg");

    piTab.add("Processed Video", processedVideo).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0).withSize(4, 3);

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

  }
}
