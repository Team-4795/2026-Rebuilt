package frc.robot.subsystems.StateManager;

import edu.wpi.first.math.geometry.Pose2d;

public enum State {
  SHOOTING(StateConstants.hub),
  SHUTTLING_LEFT_SIDE(StateConstants.shuttleLeftTarget),
  SHUTTLING_RIGHT_SIDE(StateConstants.shuttleRightTarget),
  SHUTTLING_DEAD_ZONE(new Pose2d());

  public Pose2d targetPose;

  private State(Pose2d target) {
    this.targetPose = target;
  }
}
