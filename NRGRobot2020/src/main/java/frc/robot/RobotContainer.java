package frc.robot;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.commands.ManualDriveStraight;
import frc.robot.commands.ManualFeeder;
import frc.robot.commands.ManualHood;
import frc.robot.commandSequences.AutoDriveToFuelCell;
import frc.robot.commandSequences.AutoDriveToLoadingStation;
import frc.robot.commandSequences.AutoShootSequence;
import frc.robot.commandSequences.InitiationLineToRightTrenchAuto;
import frc.robot.commands.DriveToFuelCell;
import frc.robot.commands.HoldHoodDown;
import frc.robot.commands.InterruptAll;
import frc.robot.commands.LEDTest;
import frc.robot.commands.ManualAcquirer;
import frc.robot.commands.ToggleAcquirerPiston;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.ManualTurret;
import frc.robot.commandSequences.PrepareForMatch;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetStartPosition;
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.AutoTurret;
import frc.robot.utilities.NRGPreferences;
import frc.robot.subsystems.BallCounter;
import frc.robot.subsystems.AcquirerPiston.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  // Joysticks and JoystickButtons
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);

  private JoystickButton driveStraight = new JoystickButton(leftJoystick, 1);
  private JoystickButton interruptAllButton = new JoystickButton(leftJoystick, 2);
  private JoystickButton ledModeButton = new JoystickButton(leftJoystick, 8);

  private JoystickButton shiftGears = new JoystickButton(rightJoystick, 1);
  private JoystickButton driveToBall = new JoystickButton(rightJoystick, 3);
  private JoystickButton holdHoodDownButton = new JoystickButton(rightJoystick, 4);
  private JoystickButton driveToBallContinuous = new JoystickButton(rightJoystick, 4);
  private JoystickButton driveToLoadingStation = new JoystickButton(rightJoystick, 6);
  private JoystickButton activateAcquirerPiston = new JoystickButton(rightJoystick, 10);
  private JoystickButton resetSensorsButton = new JoystickButton(rightJoystick, 11);

  // XboxController and Xbox buttons
  private XboxController xboxController = new XboxController(2);
  private JoystickButton xboxButtonA = new JoystickButton(xboxController, 1); // A Button
  private JoystickButton xboxButtonB = new JoystickButton(xboxController, 2); // B Button
  private JoystickButton xboxButtonX = new JoystickButton(xboxController, 3); // X Button
  private JoystickButton xboxButtonY = new JoystickButton(xboxController, 4); // Y button
  private JoystickButton xboxLeftBumper = new JoystickButton(xboxController, 5); 
  private JoystickButton xboxRightBumper = new JoystickButton(xboxController, 6);
  private JoystickButton xboxBackButton = new JoystickButton(xboxController, 7);

  // D-pad left/right - turret rotate
  // D-pad up/down - hood up/down
  // Xbox right trigger - manual shooter rpm
  // Xbox right stick up/down - acquirer, back button + right stick up/down - feeder,
  
  // Create subsystems
  private final RobotSubsystems subsystems = new RobotSubsystems();
  private final Compressor compressor = new Compressor();

  // Commands
  private final ManualDrive manualDrive = new ManualDrive(subsystems.drive, leftJoystick, rightJoystick, xboxController);
  private final ManualAcquirer manualAcquirer = new ManualAcquirer(subsystems.acquirer, xboxController);
  private final ManualFeeder manualFeeder = new ManualFeeder(subsystems.feeder, xboxController);
  private final ManualTurret manualTurret = new ManualTurret(subsystems.turret, xboxController);
  private final ManualHood manualHood = new ManualHood(subsystems.hood, xboxController);
  private ManualShooter manualShooter = new ManualShooter(subsystems.shooterRPM, xboxController);
  private LEDTest ledTest = new LEDTest(subsystems.leds);
  private InterruptAll interruptAll = new InterruptAll(subsystems);
  private final BallCounter ballCounter = new BallCounter();

  // Autonomous chooser
  private SendableChooser<InitialAutoPath> autoPathChooser;

  private enum InitialAutoPath {
    INITIATION_LINE_TO_MIDDLE,
    INITIATION_LINE_TO_LEFT_TRENCH,
    INITIATION_LINE_TO_RIGHT_TRENCH;
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println("RobotContainer");
    NRGPreferences.init();

    // Set subsystem default commands
    subsystems.drive.setDefaultCommand(manualDrive);
    subsystems.shooterRPM.setDefaultCommand(manualShooter);
    subsystems.acquirer.setDefaultCommand(manualAcquirer);
    subsystems.feeder.setDefaultCommand(manualFeeder);
    // turret.setDefaultCommand(manualTurret);
    subsystems.hood.setDefaultCommand(manualHood);
    // leds.setDefaultCommand(ledTest);

    // Configure the button bindings
    configureButtonBindings();

    // Configure Shuffleboard Tabs
    this.addAutonomousShuffleboardTab();
    this.addDriverShuffleboardTab();
    subsystems.drive.addShuffleBoardTab();
    subsystems.raspPi.addShuffleBoardTab();
    subsystems.acquirer.initShuffleboard();
    subsystems.feeder.initShuffleboard();
    subsystems.turret.initShuffleboard();
    subsystems.hood.initShuffleboard();
    subsystems.ballCounter.addShuffleboardTab();
    subsystems.shooterRPM.addShuffleBoardTab();

    compressor.start();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings(){
    xboxButtonB.whenPressed(new MaintainShooterRPM(subsystems.shooterRPM));
    xboxButtonX.whenPressed(new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND).alongWith(new TurnTurretToAngle(subsystems.turret, 77)).alongWith(new DriveToFuelCell(subsystems.drive, subsystems.raspPi)));
    xboxButtonX.whenHeld(new AutoFeeder(subsystems.ballCounter, subsystems.feeder).alongWith(
      new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withAbsoluteCount(5)));
      xboxButtonX.whenReleased(new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT));
      xboxLeftBumper.whenPressed(new AutoTurret(subsystems.turret, subsystems.limelightVision));
      xboxRightBumper.whenPressed(new AutoShootSequence(4000, subsystems));
      xboxBackButton.whenPressed(new ManualTurret(subsystems.turret, xboxController));
      driveStraight.whenHeld(new ManualDriveStraight(subsystems.drive, leftJoystick));
      shiftGears.whenPressed(new InstantCommand(() -> { subsystems.gearbox.toggleGears(); } ));
      activateAcquirerPiston.whenPressed(new ToggleAcquirerPiston(subsystems.acquirerPiston));
      resetSensorsButton.whenPressed(new InstantCommand(() -> {
        resetSensors();
      }));
      ledModeButton.whenPressed(new InstantCommand(() -> {
        subsystems.limelightVision.toggleLed();
      }));
    driveToBall.whenPressed(new AutoDriveToFuelCell(subsystems, 1));
    driveToLoadingStation.whenPressed(new AutoDriveToLoadingStation(subsystems.raspPi, subsystems.drive, 0.0, 0.0));
    driveToBallContinuous.whenPressed(new DriveToFuelCell(subsystems.drive, subsystems.raspPi));
    interruptAllButton.whenPressed(interruptAll);
    holdHoodDownButton.whenHeld(new HoldHoodDown(subsystems.hood));
  }
  
  /**
   * Adds buttons to the shuffleboard for the driver
   */
  private void addDriverShuffleboardTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    CameraServer cs = CameraServer.getInstance();

    // Initialize the video sources and create a switched camera.
    HttpCamera processedVideo = new HttpCamera("Processed", "http://frcvision.local:1181/stream.mjpg");
    processedVideo.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    HttpCamera limelightVideo = new HttpCamera("limelight", "http://limelight.local:5800/stream/mjpg");
    limelightVideo.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    MjpegServer switchedCamera = cs.addSwitchedCamera("Switched");
    switchedCamera.setSource(processedVideo);

    // Create a layout and add buttons to select the source of the switched camera.
    ShuffleboardLayout videoToggleLayout = driverTab.getLayout("Video Toggle", BuiltInLayouts.kList).withPosition(0,0).withSize(2, 2);
    videoToggleLayout.add("Processed", new InstantCommand(() -> switchedCamera.setSource(processedVideo)));
    videoToggleLayout.add("limelight", new InstantCommand(() -> switchedCamera.setSource(limelightVideo)));  

    // Add the switched camera to the Shuffleboard tab.
    HttpCamera switchedVideo = new HttpCamera("Switched", "http://localhost/stream/mjpg");
    driverTab.add("Switched Video", switchedVideo).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0).withSize(4, 3);
    
    ShuffleboardLayout loadingStationLayout = driverTab.getLayout("Loading Station Picker", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 0);
    loadingStationLayout.add("Drive to right feeder", new AutoDriveToLoadingStation(
        subsystems.raspPi, subsystems.drive, Units.inchesToMeters(-11), Units.inchesToMeters(22)));
    loadingStationLayout.add("Drive to left feeder", new AutoDriveToLoadingStation(
      subsystems.raspPi, subsystems.drive, Units.inchesToMeters(-11), Units.inchesToMeters(-22)));
    loadingStationLayout.add("Drive to center feeder", new AutoDriveToLoadingStation(subsystems.raspPi, subsystems.drive, 0.0, 0.0));

    ShuffleboardLayout layout = driverTab.getLayout("Ball counter",  BuiltInLayouts.kList).withPosition(0, 0)
    .withSize(2, 3);  
    layout.addNumber("Ball Count", () -> ballCounter.getBallCount());
  }

  /**
   * Adds the Shuffleboard tab for autonomous selection.
   */
  private void addAutonomousShuffleboardTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    // Create a list layout and add the autonomous selection widgets
    ShuffleboardLayout autoLayout = autoTab.getLayout("Autonomous", BuiltInLayouts.kList).withPosition(0, 0).withSize(6, 4);

    autoPathChooser = new SendableChooser<InitialAutoPath>();
    autoPathChooser.addOption(InitialAutoPath.INITIATION_LINE_TO_MIDDLE.name(), InitialAutoPath.INITIATION_LINE_TO_MIDDLE);
    autoPathChooser.addOption(InitialAutoPath.INITIATION_LINE_TO_LEFT_TRENCH.name(), InitialAutoPath.INITIATION_LINE_TO_LEFT_TRENCH);
    autoPathChooser.addOption(InitialAutoPath.INITIATION_LINE_TO_RIGHT_TRENCH.name(), InitialAutoPath.INITIATION_LINE_TO_RIGHT_TRENCH);
    autoLayout.add("Initiation Line Path", autoPathChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
    PrepareForMatch pForMatch = new PrepareForMatch(subsystems.hood, subsystems.turret, subsystems.acquirerPiston);
    autoTab.add("PrepareForMatch", pForMatch);
  }

  /**
   * Pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    resetSensors();
    InitialAutoPath path = autoPathChooser.getSelected();
    switch(path){
      case INITIATION_LINE_TO_RIGHT_TRENCH:
        return new SetStartPosition(subsystems.drive, InitiationLineToRightTrenchAuto.INITIAL_POSITION)
          .andThen(new InitiationLineToRightTrenchAuto(subsystems));
      default:
        // TODO move off of Initiation Line
        return new SetStartPosition(subsystems.drive, new Pose2d(0.0, 0.0, new Rotation2d(0)));
          }    
  }

  public void resetSensors() {
    subsystems.drive.resetHeading();
    subsystems.drive.resetOdometry(new Pose2d(0,0, new Rotation2d()));
    subsystems.shooterRPM.reset();
    subsystems.turret.resetHeading();
    subsystems.hood.reset();
  }
}