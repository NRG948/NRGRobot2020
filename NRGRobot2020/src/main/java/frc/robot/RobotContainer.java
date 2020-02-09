package frc.robot;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ManualDriveStraight;
import frc.robot.commands.ManualFeeder;
import frc.robot.commands.ManualHood;
import frc.robot.commands.ManualIndexer;
import frc.robot.commands.AutoDriveOnHeading;
import frc.robot.commands.DriveToFuelCell;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.ManualAcquirer;
import frc.robot.commands.ManualAcquirerPiston;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.ManualTurret;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.subsystems.Acquirer;
import frc.robot.subsystems.AcquirerPiston;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.NRGPreferences;
import frc.robot.subsystems.Turret;
import frc.robot.vision.FuelCellTarget;
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

  private static final int lightRelayPort = 0;
  // subsystems
  private final Drive drive = new Drive();
  private final Acquirer acquirer = new Acquirer();
  private final Indexer indexer = new Indexer();
  private final Feeder feeder = new Feeder();
  private final Turret turret = new Turret();
  private final Hood hood = new Hood();
  public final ShooterRPM shooterRPM = new ShooterRPM();
  private final LimelightVision limelightVision = new LimelightVision();
  private final RaspberryPiVision raspPi = new RaspberryPiVision();
  private final AcquirerPiston acquirerPiston = new AcquirerPiston();

  //Joystick and JoystickButtons
  private final Joystick rightJoystick = new Joystick(0);
  private final Joystick leftJoystick = new Joystick(1);
  private JoystickButton resetSensorsButton = new JoystickButton(rightJoystick, 11);
  private JoystickButton driveToBall = new JoystickButton(rightJoystick, 3);
  private JoystickButton driveToBallContinuous = new JoystickButton(rightJoystick, 4);
  private JoystickButton DriveStraight = new JoystickButton(leftJoystick, 1);
  private JoystickButton activateAcquirerPiston = new JoystickButton(rightJoystick, 10);

  //XboxController and Xbox buttons
  private XboxController xboxController = new XboxController(2);
  private JoystickButton xboxButtonA = new JoystickButton(xboxController, 1); // A Button
  private JoystickButton xboxButtonB = new JoystickButton(xboxController, 2); // B Button
  private JoystickButton xboxButtonX = new JoystickButton(xboxController, 3); // X Button
  private JoystickButton xboxButtonY = new JoystickButton(xboxController, 4); // Y button

  private Relay cameraLights;

 
  //left/right dpad - turret, up/down dpad - hood, left stick up/down - indexer, right stick up/down - acquirer, back button + right stick up/down - feeder, right trigger - shooter rpm
  //commands
  private final ManualDrive manualDrive = new ManualDrive(drive, leftJoystick, rightJoystick, xboxController);
  private final ManualAcquirer manualAcquirer = new ManualAcquirer(acquirer, xboxController);
  private final ManualIndexer manualIndexer = new ManualIndexer(indexer, xboxController);
  private final ManualFeeder manualFeeder = new ManualFeeder(feeder, xboxController);
  private final ManualTurret manualTurret = new ManualTurret(turret, xboxController);
  private final ManualHood manualHood = new ManualHood(hood, xboxController);
  private final ManualAcquirerPiston manualAcquirerPiston = new ManualAcquirerPiston(acquirerPiston, activateAcquirerPiston);
  private SetShooterRPM SetShooterRPM = new SetShooterRPM(1000.0, shooterRPM);
  private ManualShooter manualShooter = new ManualShooter(shooterRPM, xboxController);
  private FollowWaypoints followWaypointsSCurve = new FollowWaypoints(drive, new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1, -1), new Translation2d(2, 1)), new Pose2d(3, 0, new Rotation2d(0)));
  private FollowPathWeaverFile followPathTest;
 
  //autonomous chooser
  private SendableChooser<AutoPath> autoPathChooser;
  private enum AutoPath {
    INITIATION_LINE_TO_MIDDLE("INITIATION_LINE_TO_MIDDLE.wpilib.json",
      new Pose2d(3.362,-3.989, new Rotation2d(0))),
    INITIATION_LINE_TO_LEFT_TRENCH("INITIATION_LINE_TO_LEFT_TRENCH.wpilib.json",
      new Pose2d(3.3,-0.786 ,new Rotation2d(0))),
    INITIATION_LINE_TO_RIGHT_TRENCH("INITIATION_LINE_TO_RIGHT_TRENCH.wpilib.json",
      new Pose2d(3.473,-7.501,new Rotation2d(0)));
    private final String fileName;
    private final Pose2d startingPosition;
    private AutoPath(String file, Pose2d position){
      fileName = file;
      startingPosition = position;
    }
    public String getFile(){
      return fileName;
    }
    public Pose2d getStartingPosition(){
      return startingPosition;
    }
  }
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    System.out.println("RobotContainer");
    NRGPreferences.init();

    //subsystem default commands
    drive.setDefaultCommand(manualDrive);
    shooterRPM.setDefaultCommand(manualShooter);
    acquirer.setDefaultCommand(manualAcquirer);
    indexer.setDefaultCommand(manualIndexer);
    feeder.setDefaultCommand(manualFeeder);
    turret.setDefaultCommand(manualTurret);
    hood.setDefaultCommand(manualHood);
    acquirerPiston.setDefaultCommand(manualAcquirerPiston);

    // Configure the button bindings
    configureButtonBindings();
    drive.addShuffleBoardTab();
    raspPi.addShuffleBoardTab();
    try {
      followPathTest = new FollowPathWeaverFile(drive, "Test.wpilib.json");
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    // Adds AutoPath chooser to SmartDashBoard
    ShuffleboardTab autoTab = Shuffleboard.getTab("Auto");
    autoPathChooser = new SendableChooser<AutoPath>();
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_MIDDLE.name(), AutoPath.INITIATION_LINE_TO_MIDDLE);
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_LEFT_TRENCH.name(), AutoPath.INITIATION_LINE_TO_LEFT_TRENCH);
    autoPathChooser.addOption(AutoPath.INITIATION_LINE_TO_RIGHT_TRENCH.name(), AutoPath.INITIATION_LINE_TO_RIGHT_TRENCH);
    autoTab.add("autoPath",autoPathChooser);

    cameraLights = new Relay(lightRelayPort);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxButtonA.whenPressed(new InstantCommand(()-> {
      cameraLights.set(Relay.Value.kForward);
    }));
    xboxButtonB.whenPressed(new SetShooterRPM(3900, shooterRPM));
    xboxButtonY.whenPressed(followWaypointsSCurve);
    // xboxButtonA.whenPressed(new TurnTurretToTarget(limelightVision, turret));
    DriveStraight.whenHeld(new ManualDriveStraight(drive, leftJoystick));
    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      resetSensors();
    }));
    driveToBall.whenPressed(() -> {
      FuelCellTarget ballTarget = raspPi.getFuelCellTarget();
      if (ballTarget != null) {
        double distanceToTarget = ballTarget.distanceToTarget();
        double angleToTarget = ballTarget.getAngleToTarget();
        
        new AutoTurnToHeading(this.drive).withMaxPower(0.2).toHeading(this.drive.getHeading() + angleToTarget)
            .andThen(new AutoDriveOnHeading(drive).forMeters(distanceToTarget)).schedule();
      }
    });
    driveToBallContinuous.whenPressed(new DriveToFuelCell(drive, raspPi).withMaxPower(1.0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    AutoPath path = autoPathChooser.getSelected();
    drive.resetOdometry(path.getStartingPosition());
    try {
      return new FollowPathWeaverFile(drive, path.getFile());
    } catch (IOException e) {
      e.printStackTrace();
      return null;
    }
  }
  public void resetSensors(){
    drive.resetHeading();
    drive.resetOdometry(new Pose2d(1, -3, new Rotation2d()));
    shooterRPM.reset();
  }
}