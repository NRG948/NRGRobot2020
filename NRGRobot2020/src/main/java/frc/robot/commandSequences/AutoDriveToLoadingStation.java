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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowWaypoints;
import frc.robot.commands.SetRaspberryPiPipeline;
import frc.robot.commands.WaitForNewVisionData;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.RaspberryPiVision;
import frc.robot.subsystems.RaspberryPiVision.PipelineRunner;
import frc.robot.vision.LoadingStationTarget;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoDriveToLoadingStation extends SequentialCommandGroup {
  /**
   * Creates a new AutoDriveToLoadingStation.
   */
  public AutoDriveToLoadingStation(RaspberryPiVision raspberryPiVision, Drive drive) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new SetRaspberryPiPipeline(raspberryPiVision, PipelineRunner.LOADING_STATION),
        new WaitForNewVisionData(raspberryPiVision), 
        new InstantCommand(() -> {
          LoadingStationTarget target = raspberryPiVision.getLoadingTarget();
          if (target != null) {
            Pose2d start = drive.getPose();
            System.out.println("Start " + start);
            Translation2d finalPoint = target.getFinalPoint();
            System.out.println("Final " + finalPoint);
            Translation2d waypoint = target.getWaypoint();
            System.out.println("Waypoint " + waypoint);
            Pose2d end = new Pose2d(start.getTranslation().plus(finalPoint), new Rotation2d());
            new FollowWaypoints(drive, start, List.of(waypoint), end, false).schedule();
            System.out.println("End " + end);
          }
        }));
  }
}
