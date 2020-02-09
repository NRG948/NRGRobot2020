package frc.robot.commands;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import frc.robot.subsystems.Drive;

public class FollowPathWeaverFile extends FollowTrajectory {
  /**
   * Creates a new FollowPathWeaverFile.
   * 
   * @throws IOException
   */
  public FollowPathWeaverFile(Drive drive, String fileName) throws IOException {
    super(drive, TrajectoryUtil.fromPathweaverJson(new File(new File(Filesystem.getDeployDirectory(), "output"), fileName).toPath()));
  }
}
