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
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.FollowPathWeaverFile;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.ManualShooter;
import frc.robot.commands.SetShooterRPM;
import frc.robot.commands.TurnToHeading;
import frc.robot.subsystems.BallTracker;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.LimelightVision;
import frc.robot.subsystems.ShooterRPM;
import frc.robot.utilities.NRGPreferences;
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

    resetSensorsButton.whenPressed(new InstantCommand(() -> {
      resetSensors();
    }));
    driveToBall.whenPressed(() -> {
      BallTarget ballTarget = ballTracker.getBallTarget();
      if (ballTarget != null) {
        double distanceToTarget = ballTarget.distanceToTarget();
        double angleToTarget = ballTarget.getAngleToTarget();
        
        new TurnToHeading(this.drive).withMaxPower(0.2).toHeading(this.drive.getHeadingContinuous() + angleToTarget)
            .andThen(new DriveStraightDistance(drive).forMeters(distanceToTarget)).schedule();
      }
    });
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return followPathTest;
  }

  public void resetSensors(){
    drive.resetHeading();
    drive.resetOdometry(new Pose2d(1, -3, new Rotation2d()));
    shooterRPM.reset();
  }
}