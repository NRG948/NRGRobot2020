package frc.robot.commandSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.SetAcquirerState;
import frc.robot.commands.SetHoodPosition;
import frc.robot.commands.TurnTurretToAngle;
import frc.robot.subsystems.AcquirerPiston;
import frc.robot.subsystems.AcquirerPiston.State;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Turret;

public class PrepareForMatch extends ParallelCommandGroup {
  public PrepareForMatch(Hood hood, Turret turret, AcquirerPiston acquirerPiston) {
    super(
      new TurnTurretToAngle(turret, 0),
      new SetHoodPosition(hood, 0),
      new SetAcquirerState(acquirerPiston, State.RETRACT)
      );
  }
}
