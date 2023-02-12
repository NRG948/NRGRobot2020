// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ControlPanelSpinner;
// import frc.robot.utilities.Logger;
// import edu.wpi.first.wpilibj.DriverStation;

// public class PositionControl extends CommandBase {
//   private final ControlPanelSpinner panelSpinner;
//   private double maxPower;
//   private char currentColor;
//   /**
//    * goalColor is the color that we want the field's color sensor to detect.
//    * turnToColor is the color we want our robot to turn to. turnToColor assumes 
//    * that the robot will be facing the control panel at a 90 degree angle. turnToColor
//    * is two colors away from goalColor.
//    */
//   private char goalColor;
//   private char turnToColor;
//   /**
//    * Creates a new PositionControlPanel.
//    */
//   public PositionControl(final ControlPanelSpinner panelSpinner) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.panelSpinner = panelSpinner;
//   }

//   public PositionControl withMaxPower(double maxPower) {
//     this.maxPower = maxPower;
//     return this;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     this.currentColor = panelSpinner.getColor();

//     Logger.commandInit(this, "color: " + currentColor);

//     /**
//      * The clockwise order of the colors is: Red, Yellow, Blue, Green.
//      * This method assumes that when direction is positive, the Control Panel
//      * is spinning clockwise. If the goalColor is most quickly reached by 
//      * spinning counterclockwise, then make direction negative. 
//      */
//     if (currentColor == 'R' && goalColor == 'G'
//         || currentColor == 'Y' && goalColor == 'R'
//         || currentColor == 'B' && goalColor == 'Y'
//         || currentColor == 'G' && goalColor == 'B') {
//       maxPower *= -1;
//     }
      
//     String gameMessage = DriverStation.getGameSpecificMessage();
//     if (gameMessage.length() > 0) {
//       goalColor = gameMessage.charAt(0);
//     } else {
//       goalColor = 'U';
//     }

//     char[] colors = {'R', 'Y', 'B', 'G'};
//     int index = -1;
//     for (int i = 0; i < colors.length; i++) {
//       if (colors[i] == goalColor)
//         index = i;
//     }
//     /**
//      * Unless the goalColor isn't one of RYBG, the color the robot should turn to 
//      * (assuming the robot is perpendicular of the control panel) is the color 2 colors
//      * away from the color that the field's sensor should detect.
//      */
//     if (index != -1)
//       turnToColor = colors[(index + 2) % 4]; 
//     else
//       turnToColor = 'U';
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     panelSpinner.spin(maxPower);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     panelSpinner.stopMotor();
//     Logger.commandEnd(this, interrupted);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return currentColor == turnToColor;
//   }
// }
