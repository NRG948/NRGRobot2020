/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
    super(drive, TrajectoryUtil.fromPathweaverJson(new File(Filesystem.getDeployDirectory(), fileName).toPath()));
  }
}
