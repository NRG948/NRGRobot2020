package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotSubsystems;
import frc.robot.commands.AutoFeedToShooter;
import frc.robot.commands.AutoHood;
import frc.robot.commands.AutoTurret;
import frc.robot.commands.MaintainShooterRPM;
import frc.robot.commands.WaitForMinRPM;
import frc.robot.utilities.Logger;

public class AutoShootByVaryingHood extends SequentialCommandGroup {
  /**
   * Shoots up to 5 balls in a fully autonoumous mode.
   * 
   * Uses a constant shooter RPM with a varying hood angle determined from the Limelight.
   */
  public AutoShootByVaryingHood(RobotSubsystems subsystems, double rpm){
    super(
      new MaintainShooterRPM(subsystems.shooterRPM).atRpm(rpm).setAndExit()
        .alongWith(new AutoTurret(subsystems.turret).usingLimelight(), 
                   new AutoHood(subsystems.hood, true)),
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter),
      // Are you at target rpm?
      new WaitForMinRPM(rpm, subsystems.shooterRPM),
      // Release Ball
      new AutoFeedToShooter(subsystems.acquirer, subsystems.feeder, subsystems.ballCounter)
    );
  }

  @Override
  public void initialize(){
    Logger.commandInit(this);
    super.initialize();
  }
  
  @Override
  public void end(boolean interrupted){
    super.end(interrupted);
    Logger.commandEnd(this, interrupted);
  }
}
