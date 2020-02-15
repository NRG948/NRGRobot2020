/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoDriveOnHeading;
import frc.robot.commands.AutoTurnToHeading;
import frc.robot.commands.RaspberryPiPipelines;
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
    super(new RaspberryPiPipelines(raspberryPiVision, PipelineRunner.LOADING_STATION),
        new WaitForNewVisionData(raspberryPiVision), 
        new InstantCommand(() -> {
          LoadingStationTarget target = raspberryPiVision.getLoadingTarget();
          if (target != null) {
            double angleToTarget = target.getAngleToTarget();
            double distanceToTarget = target.getDistance();
            new AutoTurnToHeading(drive).withMaxPower(0.75).toHeading(drive.getHeadingContinuous() + angleToTarget)
                .andThen(new AutoDriveOnHeading(drive).withMaxPower(0.5).forInches(distanceToTarget)).schedule();
          }
        }));
  }
}
