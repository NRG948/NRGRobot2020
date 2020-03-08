package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.AcquirerPistons;
import frc.robot.subsystems.AcquirerPistons.State;
import frc.robot.utilities.Logger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class PrepareForMatch extends ParallelCommandGroup {
  public PrepareForMatch(Hood hood, Turret turret, AcquirerPistons acquirerPiston) {
    super(
      new TurnTurretToAngle(turret, 0),
      new SetHoodPosition(hood, 0),
      new SetAcquirerState(acquirerPiston, State.RETRACT)
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
    Logger.commandEnd(this);
  }
}
