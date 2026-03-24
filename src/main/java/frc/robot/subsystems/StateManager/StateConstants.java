package frc.robot.subsystems.StateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public class StateConstants {
  public static final Pose2d hub =
      new Pose2d(
          Constants.FieldConstants.blueHub.getX(),
          Constants.FieldConstants.blueHub.getY(),
          new Rotation2d());

  public static final Pose2d shuttleLeftTarget =
      new Pose2d(
          Constants.FieldConstants.shuttleTargetOne.getX(),
          Constants.FieldConstants.shuttleTargetOne.getY(),
          new Rotation2d());

  public static final Pose2d shuttleRightTarget =
      new Pose2d(
          Constants.FieldConstants.shuttleTargetTwo.getX(),
          Constants.FieldConstants.shuttleTargetTwo.getY(),
          new Rotation2d());
}
