package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.utilities.NRGPreferences;

public class FollowTrajectory extends RamseteCommand {
  /**
   * Creates a new FollowTrajectory.
   */
  public FollowTrajectory(Drive drive, Trajectory trajectory) {
    super(trajectory, 
        drive::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(NRGPreferences.PATH_KS_TERM.getValue(), 
        NRGPreferences.PATH_KV_TERM.getValue(), 
        NRGPreferences.PATH_KA_TERM.getValue()),
        DriveConstants.kDriveKinematics, 
        drive::getWheelSpeeds, 
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), 
        drive::tankDriveVolts, 
        drive);
  }

}
