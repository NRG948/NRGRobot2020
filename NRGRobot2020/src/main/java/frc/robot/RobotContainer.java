package frc.robot;

import java.io.IOException;

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
import frc.robot.commands.ManualDriveStraight;
import frc.robot.commands.ManualFeeder;
import frc.robot.commands.ManualHood;
import frc.robot.commandSequences.AutoDriveToFuelCell;
import frc.robot.commandSequences.AutoDriveToLoadingStation;
import frc.robot.commands.DriveToFuelCell;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.InterruptAll;
import frc.robot.commands.LEDTest;
import frc.robot.commands.ManualAcquirer;
import frc.robot.commands.ToggleAcquirerPiston;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.SetStartPosition;
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.AutoTurret;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.AcquirerPiston;
import frc.robot.subsystems.AddressableLEDs;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Gearbox;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.NRGPreferences;
import frc.robot.subsystems.Turret;
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

  // Create subsystems
  private final AddressableLEDs leds = new AddressableLEDs();
  private final Drive drive = new Drive();
  private final Gearbox gearbox = new Gearbox();
  private final Acquirer acquirer = new Acquirer();
  private final AcquirerPiston acquirerPiston = new AcquirerPiston();
  private final Feeder feeder = new Feeder();
  private final LimelightVision limelightVision = new LimelightVision();
  private final Turret turret = new Turret(limelightVision);
  private final Hood hood = new Hood();
  public final ShooterRPM shooterRPM = new ShooterRPM();
  private final RaspberryPiVision raspPi = new RaspberryPiVision();
  private final Compressor compressor = new Compressor();

  // Joysticks and JoystickButtons
  private final Joystick leftJoystick = new Joystick(0);
  private final Joystick rightJoystick = new Joystick(1);

  private JoystickButton driveStraight = new JoystickButton(leftJoystick, 1);
  private JoystickButton interruptAllButton = new JoystickButton(leftJoystick, 2);
  private JoystickButton ledModeButton = new JoystickButton(leftJoystick, 8);

  private JoystickButton shiftGears = new JoystickButton(rightJoystick, 1);
  private JoystickButton driveToBall = new JoystickButton(rightJoystick, 3);
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

  // D-pad left/right - turret rotate
  // D-pad up/down - hood up/down
  // Xbox right trigger - manual shooter rpm
  // Xbox right stick up/down - acquirer, back button + right stick up/down - feeder,
  
  // Commands
  private final ManualDrive manualDrive = new ManualDrive(drive, leftJoystick, rightJoystick, xboxController);
  private final ManualAcquirer manualAcquirer = new ManualAcquirer(acquirer, xboxController);
  private final ManualFeeder manualFeeder = new ManualFeeder(feeder, xboxController);
  private final ManualTurret manualTurret = new ManualTurret(turret, xboxController);
  private final ManualHood manualHood = new ManualHood(hood, xboxController);
  private ManualShooter manualShooter = new ManualShooter(shooterRPM, xboxController);
  private LEDTest ledTest = new LEDTest(leds);
  private InterruptAll interruptAll = new InterruptAll(leds, drive, acquirer, feeder,
  limelightVision, turret, hood, shooterRPM, raspPi, acquirerPiston );

  // Autonomous chooser
  private SendableChooser<AutoPath> autoPathChooser;

  private enum AutoPath {
    INITIATION_LINE_TO_MIDDLE("INITIATION_LINE_TO_MIDDLE.wpilib.json", new Pose2d(3.362, -3.989, new Rotation2d(0))),
    INITIATION_LINE_TO_LEFT_TRENCH("INITIATION_LINE_TO_LEFT_TRENCH.wpilib.json",
        new Pose2d(3.3, -0.786, new Rotation2d(0))),
    INITIATION_LINE_TO_RIGHT_TRENCH("INITIATION_LINE_TO_RIGHT_TRENCH.wpilib.json",
        new Pose2d(3.473, -7.501, new Rotation2d(0)));

    private final String fileName;
    private final Pose2d startingPosition;

    private AutoPath(String file, Pose2d position) {
      fileName = file;
      startingPosition = position;
    }

    public String getFile() {
      return fileName;
    }

    public Pose2d getStartingPosition() {
      return startingPosition;
    }
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println("RobotContainer");
    NRGPreferences.init();

    // Set subsystem default commands
    drive.setDefaultCommand(manualDrive);
    shooterRPM.setDefaultCommand(manualShooter);
    acquirer.setDefaultCommand(manualAcquirer);
    feeder.setDefaultCommand(manualFeeder);
    turret.setDefaultCommand(manualTurret);
    hood.setDefaultCommand(manualHood);
    // leds.setDefaultCommand(ledTest);

    // Configure the button bindings
    configureButtonBindings();

    // Configure Shuffleboard Tabs
    this.addAutonomousShuffleboardTab();
    drive.addShuffleBoardTab();
    raspPi.addShuffleBoardTab();
    acquirer.initShuffleboard();
    feeder.initShuffleboard();
    turret.initShuffleboard();
    hood.initShuffleboard();

    compressor.start();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxButtonB.whenPressed(new MaintainShooterRPM(shooterRPM));
    xboxLeftBumper.whenPressed(new AutoTurret(turret));
    driveStraight.whenHeld(new ManualDriveStraight(drive, leftJoystick));
    shiftGears.whenPressed(new InstantCommand(() -> { gearbox.toggleGears(); } ));
    activateAcquirerPiston.whenPressed(new ToggleAcquirerPiston(acquirerPiston));
    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      resetSensors();
    }));
    ledModeButton.whenPressed(new InstantCommand(() -> {
      limelightVision.toggleLed();
    }));
    driveToBall.whenPressed(new AutoDriveToFuelCell(this.raspPi, this.drive));
    driveToLoadingStation.whenPressed(new AutoDriveToLoadingStation(this.raspPi, this.drive));
    driveToBallContinuous.whenPressed(new DriveToFuelCell(drive, raspPi));
    interruptAllButton.whenPressed(interruptAll);
  }

  /**
   * Adds the Shuffleboard tab for autonomous selection.
   */
  private void addAutonomousShuffleboardTab() {
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    // Create a list layout and add the autonomous selection widgets
    ShuffleboardLayout autoLayout = autoTab.getLayout("Autonomous", BuiltInLayouts.kList).withPosition(0, 0).withSize(6, 4);

    autoPathChooser = new SendableChooser<AutoPath>();
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_MIDDLE.name(), AutoPath.INITIATION_LINE_TO_MIDDLE);
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_LEFT_TRENCH.name(), AutoPath.INITIATION_LINE_TO_LEFT_TRENCH);
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_RIGHT_TRENCH.name(), AutoPath.INITIATION_LINE_TO_RIGHT_TRENCH);
    autoLayout.add("Initiation Line Path", autoPathChooser).withWidget(BuiltInWidgets.kSplitButtonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    AutoPath path = autoPathChooser.getSelected();
    new SetStartPosition(drive, path.getStartingPosition());
    
    try {
      return new FollowPathWeaverFile(drive, path.getFile());
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  public void resetSensors() {
    drive.resetHeading();
    drive.resetOdometry(new Pose2d(0,0, new Rotation2d()));
    shooterRPM.reset();
  }
}