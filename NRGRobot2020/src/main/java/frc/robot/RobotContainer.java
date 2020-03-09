package frc.robot;

import static frc.robot.utilities.NRGPreferences.HOOD_POSITION_INITIATION;
import static frc.robot.utilities.NRGPreferences.HOOD_POSITION_TRENCH_NEAR;
import static frc.robot.utilities.NRGPreferences.HOOD_POSITION_TRENCH_FAR;
import static frc.robot.utilities.NRGPreferences.SHOOTER_RPM_INITIATION;
import static frc.robot.utilities.NRGPreferences.SHOOTER_RPM_TRENCH_NEAR;
import static frc.robot.utilities.NRGPreferences.SHOOTER_RPM_TRENCH_FAR;

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
import frc.robot.commandSequences.InitiationLineRollForward;
import frc.robot.commandSequences.InitiationLineToLeftTrenchAuto;
import frc.robot.commandSequences.InitiationLineToRightTrenchAuto;
import frc.robot.commandSequences.InitiationLineToShieldGeneratorAuto;
import frc.robot.commands.DriveToFuelCell;
import frc.robot.commands.InterruptAll;
import frc.robot.commands.LEDTest;
import frc.robot.commands.ManualAcquirer;
import frc.robot.commands.TurnClimberWinch;
import frc.robot.commands.ToggleAcquirerPiston;
import frc.robot.commands.ToggleClimberPiston;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.ManualTurret;
import frc.robot.commandSequences.PrepareForMatch;
import frc.robot.commandSequences.StopAutoShootSequence;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.SetLimelightHorizontalSkew;
import frc.robot.commands.SetStartPosition;
import frc.robot.commands.StopTurretAnglePID;
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.AcquireNumberOfBalls;
import frc.robot.commands.AutoFeeder;
import frc.robot.commands.AutoTurret;
import frc.robot.commands.Delay;
import frc.robot.commands.DisableShooterRPM;
import frc.robot.utilities.NRGPreferences;
import frc.robot.subsystems.ClimberPiston;
import frc.robot.subsystems.AcquirerPistons.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  private JoystickButton leftJoyButton1 = new JoystickButton(leftJoystick, 1);
  private JoystickButton leftJoyButton2 = new JoystickButton(leftJoystick, 2);
  private JoystickButton leftJoyButton3 = new JoystickButton(leftJoystick, 3);
  private JoystickButton leftJoyButton4 = new JoystickButton(leftJoystick, 4);
  private JoystickButton leftJoyButton8 = new JoystickButton(leftJoystick, 8);

  private JoystickButton rightJoyButton1 = new JoystickButton(rightJoystick, 1);
  private JoystickButton rightJoyButton3 = new JoystickButton(rightJoystick, 3);
  private JoystickButton rightJoyButton4 = new JoystickButton(rightJoystick, 4);
  private JoystickButton rightJoyButton5 = new JoystickButton(rightJoystick, 5);
  private JoystickButton rightJoyButton6 = new JoystickButton(rightJoystick, 6);
  private JoystickButton rightJoyButton10 = new JoystickButton(rightJoystick, 10);
  private JoystickButton rightJoyButton11 = new JoystickButton(rightJoystick, 11);

  // XboxController and Xbox buttons
  private XboxController xboxController = new XboxController(2);
  private JoystickButton xboxButtonA = new JoystickButton(xboxController, 1); // A Button
  private JoystickButton xboxButtonB = new JoystickButton(xboxController, 2); // B Button
  private JoystickButton xboxButtonX = new JoystickButton(xboxController, 3); // X Button
  private JoystickButton xboxButtonY = new JoystickButton(xboxController, 4); // Y button
  private JoystickButton xboxLeftBumper = new JoystickButton(xboxController, 5);
  private JoystickButton xboxRightBumper = new JoystickButton(xboxController, 6);
  private JoystickButton xboxBackButton = new JoystickButton(xboxController, 7);
  private JoystickButton xboxButton9 = new JoystickButton(xboxController, 9);
  private JoystickButton xboxButton10 = new JoystickButton(xboxController, 10);
  // D-pad left/right - turret rotate
  // D-pad up/down - hood up/down
  // Xbox right trigger - manual shooter rpm
  // Xbox right stick up/down - acquirer, 
  // back button + right stick up/down - feeder,

  // Create subsystems
  private final RobotSubsystems subsystems = new RobotSubsystems();
  private final Compressor compressor = new Compressor();

  // Commands
  private final ManualDrive manualDrive = new ManualDrive(subsystems.drive, leftJoystick, rightJoystick, xboxController);
  private final ManualAcquirer manualAcquirer = new ManualAcquirer(subsystems.acquirer, xboxController);
  private final ManualFeeder manualFeeder = new ManualFeeder(subsystems.feeder, xboxController);
  private final ManualTurret manualTurret = new ManualTurret(subsystems.turret, xboxController);
  private final ManualHood manualHood = new ManualHood(subsystems.hood, xboxController);
  private final ManualShooter manualShooter = new ManualShooter(subsystems.shooterRPM, xboxController);
  private final AutoShootSequence shootFromInitiation = new AutoShootSequence(subsystems, SHOOTER_RPM_INITIATION.getValue(),  HOOD_POSITION_INITIATION.getValue(),   0.0);
  private final AutoShootSequence shootFromTrenchNear = new AutoShootSequence(subsystems, SHOOTER_RPM_TRENCH_NEAR.getValue(), HOOD_POSITION_TRENCH_NEAR.getValue(), -1.5);
  private final AutoShootSequence shootFromTrenchFar  = new AutoShootSequence(subsystems, SHOOTER_RPM_TRENCH_FAR.getValue(),  HOOD_POSITION_TRENCH_FAR.getValue(), -1.0);
  private final CommandBase stopAutoShootSequence = new StopAutoShootSequence(subsystems);

  private final LEDTest ledTest = new LEDTest(subsystems.leds);
  private final InterruptAll interruptAll = new InterruptAll(subsystems);

  // When we press down the HoldHoodDownButton we store the original hood position here.
  private double originalHoodPosition;

  // Autonomous chooser
  private SendableChooser<InitialAutoPath> autoPathChooser;
  private SendableChooser<InitialDelay> autoDelay;

  private enum InitialAutoPath {
    INITIATION_LINE_TO_LEFT_TRENCH, 
    INITIATION_LINE_TO_RIGHT_TRENCH, 
    INITIATION_LINE_TO_SHIELD_GENERATOR, 
    INITIATION_LINE_ROLL_FORWARD
  }

  private enum InitialDelay {
    DELAY_0, DELAY_2, DELAY_5
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
    subsystems.limelightVision.addShuffleboardTab();
    
    compressor.start();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /*
     * Xbox controller button mappings
     */
    xboxButtonA.whenPressed(new ToggleAcquirerPiston(subsystems.acquirerPiston));
    xboxButtonB.whenPressed(new MaintainShooterRPM(subsystems.shooterRPM));
    
    // Holding x button activates AutoDriveToFuelcellsSequence
    xboxButtonX.whenPressed(new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND)
      .alongWith(new TurnTurretToAngle(subsystems.turret, 77), new DriveToFuelCell(subsystems.drive, subsystems.raspPi)));
    xboxButtonX.whenHeld(new AutoFeeder(subsystems.ballCounter, subsystems.feeder).alongWith(
      new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withAbsoluteCount(4)));
    xboxButtonX.whenReleased(new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT));

    // Holding x button activates AcquirerSequence
    xboxButtonY.whenPressed(new SetAcquirerState(subsystems.acquirerPiston, State.EXTEND)
      .alongWith(new TurnTurretToAngle(subsystems.turret, 77)));
    xboxButtonY.whenHeld(new AutoFeeder(subsystems.ballCounter, subsystems.feeder).alongWith(
      new AcquireNumberOfBalls(subsystems.acquirer, subsystems.ballCounter).withAbsoluteCount(4)));
    xboxButtonY.whenReleased(new SetAcquirerState(subsystems.acquirerPiston, State.RETRACT));
    
    xboxLeftBumper.whenPressed(new AutoTurret(subsystems.turret).usingLimelight());
    xboxRightBumper.whenHeld(shootFromInitiation);
    xboxRightBumper.whenReleased(stopAutoShootSequence);
    // xboxRightBumper.whenPressed(shootFromTrenchNear);
    // xboxRightBumper.whenPressed(shootFromTrenchFar);
    xboxBackButton.whenPressed(manualTurret);
    xboxButton9.whenPressed( () -> subsystems.ballCounter.addToBallCount(-1));
    xboxButton10.whenPressed( () -> subsystems.ballCounter.addToBallCount(1));

     /*
     * Joystick button mappings.
     */
    leftJoyButton1.whenHeld(new ManualDriveStraight(subsystems.drive, leftJoystick));
    rightJoyButton1.whenPressed( () -> subsystems.gearbox.toggleGears());
    rightJoyButton10.whenPressed(new ToggleAcquirerPiston(subsystems.acquirerPiston));
    rightJoyButton11.whenPressed( () -> resetSensors());
    leftJoyButton8.whenPressed( () -> subsystems.limelightVision.toggleLed());
    rightJoyButton3.whenPressed(new AutoDriveToFuelCell(subsystems, 1));
    rightJoyButton6.whenPressed(new AutoDriveToLoadingStation(subsystems.raspPi, subsystems.drive, 0.0, 0.0));
    rightJoyButton5.whenPressed(new DriveToFuelCell(subsystems.drive, subsystems.raspPi));
    leftJoyButton2.whenPressed(interruptAll);
    rightJoyButton4.whenPressed(new InstantCommand(() -> { originalHoodPosition = subsystems.hood.getPosition(); })
        .andThen(new SetHoodPosition(subsystems.hood, 2)));
    rightJoyButton4.whenReleased(() -> new SetHoodPosition(subsystems.hood, originalHoodPosition).schedule());
    leftJoyButton3.whenPressed(new ToggleClimberPiston(subsystems.climberPiston));
    leftJoyButton4.whenHeld(new InstantCommand(() -> subsystems.climberPiston.setState(ClimberPiston.State.RETRACT))
      .andThen(new TurnClimberWinch(subsystems.climberWinch).withMaxPower(0.4)));
  }
  
  /**
   * Adds buttons to the shuffleboard for the driver
   */
  private void addDriverShuffleboardTab() {
    ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");
    CameraServer cs = CameraServer.getInstance();

    // Initialize the video sources and create a switched camera.
    HttpCamera processedVideo = new HttpCamera("Processed", "http://frcvision.local:1182/stream.mjpg");
    processedVideo.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    HttpCamera limelightVideo = new HttpCamera("limelight", "http://limelight.local:5800/stream.mjpg");
    limelightVideo.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);

    MjpegServer switchedCamera = cs.addSwitchedCamera("Switched");
    switchedCamera.setSource(processedVideo);

    // Create a layout and add buttons to select the source of the switched camera.
    ShuffleboardLayout videoToggleLayout = driverTab.getLayout("Video Toggle", BuiltInLayouts.kList).withPosition(0,0).withSize(2, 2);
    videoToggleLayout.add("Processed", new InstantCommand(() -> switchedCamera.setSource(processedVideo)));
    videoToggleLayout.add("limelight", new InstantCommand(() -> switchedCamera.setSource(limelightVideo)));  

    // Add the switched camera to the Shuffleboard tab.
    HttpCamera switchedVideo = new HttpCamera("Switched", "http://localhost/stream.mjpg");
    driverTab.add("Switched Video", switchedVideo).withWidget(BuiltInWidgets.kCameraStream).withPosition(2, 0).withSize(4, 3);
    
    // TODO Remove temporary buttons to test drive to feeder.
    ShuffleboardLayout loadingStationLayout = driverTab.getLayout("Loading Station Picker", BuiltInLayouts.kList).withPosition(6, 0).withSize(2, 2);
    loadingStationLayout.add("Drive to right feeder", new AutoDriveToLoadingStation(
        subsystems.raspPi, subsystems.drive, Units.inchesToMeters(-11), Units.inchesToMeters(22)));
    loadingStationLayout.add("Drive to left feeder", new AutoDriveToLoadingStation(
      subsystems.raspPi, subsystems.drive, Units.inchesToMeters(-11), Units.inchesToMeters(-22)));
    loadingStationLayout.add("Drive to center feeder", new AutoDriveToLoadingStation(subsystems.raspPi, subsystems.drive, 0.0, 0.0));

    ShuffleboardLayout layout = driverTab.getLayout("Ball counter",  BuiltInLayouts.kList).withPosition(0, 2)
    .withSize(2, 1);  
    layout.addNumber("Ball Count", () -> subsystems.ballCounter.getBallCount());
  }

  /**
   * Adds the Shuffleboard tab for autonomous selection.
   */
  private void addAutonomousShuffleboardTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    // Create a list layout and add the autonomous selection widgets
    ShuffleboardLayout autoLayout = autoTab.getLayout("Autonomous", BuiltInLayouts.kList).withPosition(0, 0).withSize(6, 4);

    autoPathChooser = new SendableChooser<InitialAutoPath>();
    autoPathChooser.addOption(InitialAutoPath.INITIATION_LINE_TO_LEFT_TRENCH.name(), InitialAutoPath.INITIATION_LINE_TO_LEFT_TRENCH);
    autoPathChooser.addOption(InitialAutoPath.INITIATION_LINE_TO_RIGHT_TRENCH.name(), InitialAutoPath.INITIATION_LINE_TO_RIGHT_TRENCH);
    autoPathChooser.addOption(InitialAutoPath.INITIATION_LINE_TO_SHIELD_GENERATOR.name(), InitialAutoPath.INITIATION_LINE_TO_SHIELD_GENERATOR);
    autoPathChooser.addOption(InitialAutoPath.INITIATION_LINE_ROLL_FORWARD.name(), InitialAutoPath.INITIATION_LINE_ROLL_FORWARD);
    autoLayout.add("Initiation Line Path", autoPathChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
    
    // Add an optional delay before Autonomous movement
    autoDelay = new SendableChooser<InitialDelay>();
    autoDelay.addOption("0 sec", InitialDelay.DELAY_0);
    autoDelay.addOption("2 sec", InitialDelay.DELAY_2);
    autoDelay.addOption("5 sec", InitialDelay.DELAY_5);
    autoLayout.add("Delay before movement", autoDelay).withWidget(BuiltInWidgets.kSplitButtonChooser);
    // autoLayout.add("Delay before movement", autoDelay).withWidget(BuiltInWidgets.kNumberBar);

    
    PrepareForMatch pForMatch = new PrepareForMatch(subsystems.hood, subsystems.turret, subsystems.acquirerPiston);
    autoTab.add("PrepareForMatch", pForMatch);
  }

  private float getInitialDelay(){
    InitialDelay delay  = autoDelay.getSelected();
    if(delay == InitialDelay.DELAY_0){
      return 0;
    }
    else if(delay == InitialDelay.DELAY_2){
      return 2;
    }
    else{
      return 5;
    }
  }

  /**
   * Pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    resetSensors();
    InitialAutoPath path = autoPathChooser.getSelected();
    float delay  = getInitialDelay();
    switch (path) {
      case INITIATION_LINE_TO_RIGHT_TRENCH:
        return new SetStartPosition(subsystems.drive, InitiationLineToRightTrenchAuto.INITIAL_POSITION)
          .andThen(new InitiationLineToRightTrenchAuto(subsystems, delay));

      case INITIATION_LINE_TO_LEFT_TRENCH:
        return new SetStartPosition(subsystems.drive, InitiationLineToLeftTrenchAuto.INITIAL_POSITION)
          .andThen(new InitiationLineToLeftTrenchAuto(subsystems, delay));

      case INITIATION_LINE_TO_SHIELD_GENERATOR:
        return new SetStartPosition(subsystems.drive, InitiationLineToShieldGeneratorAuto.INITIAL_POSITION)
          .andThen(new InitiationLineToShieldGeneratorAuto(subsystems, delay));

      case INITIATION_LINE_ROLL_FORWARD:
        return new SetStartPosition(subsystems.drive, InitiationLineRollForward.INITIAL_POSITION)
          .andThen(new InitiationLineRollForward(subsystems, delay));
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