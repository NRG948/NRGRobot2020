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
import frc.robot.utilities.Logger;
import frc.robot.vision.LoadingStationTarget;

/**
 * Autonomous command sequence that drives to the left, center or right fuel
 * cell loading station based on vision processing input from the Raspberry Pi.
 */
public class AutoDriveToLoadingStation extends SequentialCommandGroup {
  private RaspberryPiVision raspberryPiVision;
  private Drive drive;
  private LoadingStationTarget target;
  private double xOffset;
  private double yOffset;
  private Pose2d start;
  private Pose2d end;
  private Translation2d waypoint;

  /**
   * Creates a new AutoDriveToLoadingStation.
   */
  public AutoDriveToLoadingStation(RaspberryPiVision raspberryPiVision, Drive drive, double xOffset, double yOffset) {
    this.raspberryPiVision = raspberryPiVision;
    this.drive = drive;
    this.xOffset = xOffset;
    this.yOffset = yOffset;

    addCommands(new SetRaspberryPiPipeline(raspberryPiVision, PipelineRunner.LOADING_STATION),
        new WaitForNewVisionData(raspberryPiVision).withTimeout(0.25), 
        new InstantCommand(() -> scheduleFollowWaypoints()));
  }

  @Override
  public void initialize(){
    LoadingStationTarget target = raspberryPiVision.getLoadingTarget();
    if (target == null) {
      Logger.commandInit(this, "NO TARGET");
    } else {
      double heading = drive.getHeading();
      start = drive.getPose();
      Translation2d finalPoint = target.getFinalPoint(heading, xOffset, yOffset);
      waypoint = target.getWaypoint(heading, xOffset, yOffset);
      end = new Pose2d(start.getTranslation().plus(finalPoint), new Rotation2d());
    }

    Logger.commandInit(this,
      String.format("start: %s, waypoint: %s, end: %s ", start.toString(), waypoint.toString(), end.toString()));
    super.initialize();
  }
  
  @Override
  public void end(boolean interrupted){
    super.end(interrupted);
    Logger.commandEnd(this, interrupted);
  }

  private void scheduleFollowWaypoints() {
    if (target != null) {
      new FollowWaypoints(drive, start, List.of(waypoint), end, false).schedule();
    }
  }
}
