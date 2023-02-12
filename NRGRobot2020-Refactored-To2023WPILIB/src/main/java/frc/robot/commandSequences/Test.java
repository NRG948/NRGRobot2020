// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commandSequences;

// import java.util.List;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.RobotSubsystems;
// import frc.robot.commands.FollowWaypoints;

// public class Test extends SequentialCommandGroup {
//   /**
//    * Creates a new Test.
//    */

//   public static Pose2d point1 = new Pose2d(0.0, 1.0, new Rotation2d(0));
//   public static final List<Translation2d> pointList = List.of(new Translation2d(0.0, 2.0));
//   public static Pose2d point2 = new Pose2d(0.0, 3.0, new Rotation2d(0));

//   public Test(RobotSubsystems subsystems) {
//     super(new FollowWaypoints(subsystems.drive, point1, pointList, point2, true));
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     super.initialize();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(final boolean interrupted) {
//     super.end(interrupted);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
