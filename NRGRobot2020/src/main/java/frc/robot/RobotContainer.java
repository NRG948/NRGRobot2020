/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ManualDriveStraight;
import frc.robot.commands.AutoDriveOnHeading;
import frc.robot.commands.DriveToBall;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.subsystems.BallTracker;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.NRGPreferences;
import frc.robot.subsystems.Turret;
import frc.robot.vision.BallTarget;
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

  // The robot's subsystems and commands are defined here...
  private final Drive drive = new Drive();
  private final BallTracker ballTracker = new BallTracker();

  public final ShooterRPM shooterRPM = new ShooterRPM();

  private final Joystick rightJoystick = new Joystick(0);
  private final Joystick leftJoystick = new Joystick(1);
  private JoystickButton resetSensorsButton = new JoystickButton(rightJoystick, 11);
  private JoystickButton driveToBall = new JoystickButton(rightJoystick, 3);
  private JoystickButton driveToBallContinuous = new JoystickButton(rightJoystick, 4);
  private JoystickButton DriveStraight = new JoystickButton(leftJoystick, 1);
  

  private XboxController xboxController = new XboxController(2);

  private JoystickButton xboxButtonA = new JoystickButton(xboxController, 1); // A Button
  private JoystickButton xboxButtonB = new JoystickButton(xboxController, 2); // B Button
  private JoystickButton xboxButtonX = new JoystickButton(xboxController, 3); // X Button
  private JoystickButton xboxButtonY = new JoystickButton(xboxController, 4); // Y button

  private final ManualDrive manualDrive = new ManualDrive(drive, leftJoystick, rightJoystick, xboxController);
  private SetShooterRPM SetShooterRPM = new SetShooterRPM(1000.0, shooterRPM);
  private ManualShooter manualShooter = new ManualShooter(shooterRPM, xboxController);
  private FollowWaypoints followWaypointsSCurve = new FollowWaypoints(drive, new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1, -1), new Translation2d(2, 1)), new Pose2d(3, 0, new Rotation2d(0)));
  private FollowPathWeaverFile followPathTest;
  private LimelightVision limelightVision = new LimelightVision();
  // private Turret turret = new Turret();
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
    drive.setDefaultCommand(manualDrive);
    shooterRPM.setDefaultCommand(manualShooter);
    // Configure the button bindings
    configureButtonBindings();
    drive.addShuffleBoardTab();
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
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    xboxButtonB.whenPressed(new SetShooterRPM(3900, shooterRPM));
    xboxButtonY.whenPressed(followWaypointsSCurve);
    // xboxButtonA.whenPressed(new TurnTurretToTarget(limelightVision, turret));
    DriveStraight.whenHeld(new ManualDriveStraight(drive, leftJoystick));
    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      resetSensors();
    }));
    driveToBall.whenPressed(() -> {
      BallTarget ballTarget = ballTracker.getBallTarget();
      if (ballTarget != null) {
        double distanceToTarget = ballTarget.distanceToTarget();
        double angleToTarget = ballTarget.getAngleToTarget();
        
        new AutoTurnToHeading(this.drive).withMaxPower(0.2).toHeading(this.drive.getHeading() + angleToTarget)
            .andThen(new AutoDriveOnHeading(drive).forMeters(distanceToTarget)).schedule();
      }
    });
    driveToBallContinuous.whenPressed(new DriveToBall(drive, ballTracker).withMaxPower(1.0));
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