/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.NRGPreferences;

public class FollowWaypoints extends FollowTrajectory {
  private static final int MAX_VOLTAGE = 10;
  private static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(NRGPreferences.PATH_KS_TERM.getValue(), 
          NRGPreferences.PATH_KV_TERM.getValue(), NRGPreferences.PATH_KA_TERM.getValue()),
      DriveConstants.kDriveKinematics, MAX_VOLTAGE);
  private static final TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics)
          .addConstraint(VOLTAGE_CONSTRAINT);

  /**
   * Creates a new FollowWaypoints.
   */
  public FollowWaypoints(Drive drive, Pose2d begin, List<Translation2d> waypoints, Pose2d end) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(drive, TrajectoryGenerator.generateTrajectory(begin, waypoints, end, config));
  }
}
