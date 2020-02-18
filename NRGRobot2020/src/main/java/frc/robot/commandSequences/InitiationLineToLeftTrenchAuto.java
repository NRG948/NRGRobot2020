/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandSequences;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.SetStartPosition;
import frc.robot.subsystems.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class InitiationLineToLeftTrenchAuto extends SequentialCommandGroup {
  /**
   * Creates a new InitiationLineToLeftTrenchAuto.
   */
  public InitiationLineToLeftTrenchAuto(Drive drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new SetStartPosition(drive, new Pose2d(3.3, -0.786, new Rotation2d(0))), 
      new FollowWaypoints(drive,
                          // Starting pose
                          new Pose2d( 3.3, -0.786, new Rotation2d(0)),
                          // Waypoint
                          List.of(new Translation2d(4.339,-0.86), new Translation2d(5.489, -1.651)),
                          // Ending pose
                          new Pose2d(6.243, -2.53, new Rotation2d(Math.toRadians(-90))),
                          // Drive forward
                          false));
  }
}
