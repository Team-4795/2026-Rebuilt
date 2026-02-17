package frc.robot.subsystems.StateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class StateConstants {
  public static final Pose2d hub =
      new Pose2d(
          Constants.FieldConstants.redHub.getX(),
          Constants.FieldConstants.redHub.getY(),
          new Rotation2d());

  public static final Pose2d shuttleLeftTarget =
      new Pose2d(
          Constants.FieldConstants.redShuttleTargetOne.getX(),
          Constants.FieldConstants.redShuttleTargetOne.getY(),
          new Rotation2d());

  public static final Pose2d shuttleRightTarget =
      new Pose2d(
          Constants.FieldConstants.redShuttleTargetTwo.getX(),
          Constants.FieldConstants.redShuttleTargetTwo.getY(),
          new Rotation2d());
}
