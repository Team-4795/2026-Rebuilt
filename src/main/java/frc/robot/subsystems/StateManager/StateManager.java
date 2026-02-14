package frc.robot.subsystems.StateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ShooterHood.ShooterHoodConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ZoneVisualization;
import org.littletonrobotics.junction.Logger;

public class StateManager extends SubsystemBase {
  private static StateManager instance;

  // Corners of zone as an array
  private Translation2d[] decapitationZone;
  private Translation2d[] shuttlingZone;

  // Class to visualize zones in ascope
  private ZoneVisualization decapitationVisualization = new ZoneVisualization();
  private ZoneVisualization shuttlingZoneVisualization = new ZoneVisualization();

  public static class OperationStates {
    public static boolean inDecapitationZone = false;
    public static boolean isShuttling = false;
  }

  public static StateManager initalize() {
    instance = new StateManager();
    return instance;
  }

  public static StateManager getInstance() {
    if (instance != null) {
      return instance;
    } else {
      return initalize();
    }
  }

  private StateManager() {
    this.decapitationZone = new Translation2d[4];
    this.shuttlingZone = Constants.FieldConstants.shuttleZone;

    shuttlingZoneVisualization.updateZone(shuttlingZone);
  }

  // Update zone based off the closest trench and robot velocity
  private void updateDecapitationZone() {
    Pose2d robotPose = Drive.getInstance().getPose();
    Translation2d robotTranslation = robotPose.getTranslation();

    Translation2d closest = robotTranslation.nearest(FieldConstants.trenchList);

    // box dimensions scale with velocity
    var fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(
            Drive.getInstance().getChassisSpeeds(), robotPose.getRotation());

    // dimensions of no auto score zone
    double boxXDim =
        1 + ShooterHoodConstants.boxXMultiplier * Math.abs(fieldRelative.vxMetersPerSecond);
    double boxYDim =
        FieldConstants.trenchWidth
            + ShooterHoodConstants.boxYMultiplier * Math.abs(fieldRelative.vyMetersPerSecond);

    Translation2d topLeft = new Translation2d(closest.getX() - boxXDim, closest.getY() + boxYDim);
    Translation2d topRight = new Translation2d(closest.getX() + boxXDim, closest.getY() + boxYDim);
    Translation2d botLeft = new Translation2d(closest.getX() - boxXDim, closest.getY() - boxYDim);
    Translation2d botRight = new Translation2d(closest.getX() + boxXDim, closest.getY() - boxYDim);

    decapitationZone[0] = topLeft;
    decapitationZone[1] = topRight;
    decapitationZone[2] = botLeft;
    decapitationZone[3] = botRight;

    decapitationVisualization.updateZone(decapitationZone);
  }

  @Override
  public void periodic() {
    updateDecapitationZone();

    // Visualize zones in Ascope (turn Pose2d[] into trajectories to actually see rectangle)
    decapitationVisualization.logPoints("Decapitation Zone");
    shuttlingZoneVisualization.logPoints("Shuttle Zone");

    // Update operation states based on if robot in zone
    OperationStates.inDecapitationZone =
        decapitationVisualization.isRobotInZone(Drive.getInstance().getPose());
    OperationStates.isShuttling =
        shuttlingZoneVisualization.isRobotInZone(Drive.getInstance().getPose());

    Logger.recordOutput(
        "Operation States/In Decapitation Zone", OperationStates.inDecapitationZone);
    Logger.recordOutput("Operation States/Shuttling", OperationStates.isShuttling);
  }
}
