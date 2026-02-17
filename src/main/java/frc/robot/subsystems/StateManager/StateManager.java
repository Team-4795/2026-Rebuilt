package frc.robot.subsystems.StateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ShooterHood.ShooterHoodConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.ZoneVisualization;
import java.awt.geom.Path2D;
import org.littletonrobotics.junction.Logger;

public class StateManager extends SubsystemBase {
  private static StateManager instance;
  private State state;

  // Corners of zone as an array
  private Translation2d[] decapitationZone;

  private Translation2d[] blueShuttlingZoneOne;
  private Translation2d[] blueShuttlingZoneTwo;
  private Translation2d[] blueShuttlingZoneThree;
  private Translation2d[] blueShuttlingZoneFour;
  private Translation2d[] blueShuttlingZoneFive;

  private Translation2d[] redShuttlingZoneOne;
  private Translation2d[] redShuttlingZoneTwo;
  private Translation2d[] redShuttlingZoneThree;
  private Translation2d[] redShuttlingZoneFour;
  private Translation2d[] redShuttlingZoneFive;

  // Class to visualize zones in ascope
  private Path2D decapitationVisualization;

  private Path2D blueShuttlingZoneOneVisualization;
  private Path2D blueShuttlingZoneTwoVisualization;
  private Path2D blueShuttlingZoneThreeVisualization;
  private Path2D blueShuttlingZoneFourVisualization;
  private Path2D blueShuttlingZoneFiveVisualization;

  private Path2D redShuttlingZoneOneVisualization;
  private Path2D redShuttlingZoneTwoVisualization;
  private Path2D redShuttlingZoneThreeVisualization;
  private Path2D redShuttlingZoneFourVisualization;
  private Path2D redShuttlingZoneFiveVisualization;

  private Alliance alliance;
  private Pose2d pose;

  public static class OperationStates {
    public static boolean inDecapitationZone = false;

    public static boolean inShuttlingZone1 = false;
    public static boolean inShuttlingZone2 = false;
    public static boolean inShuttlingZone3 = false;
    public static boolean inShuttlingZone4 = false;
    public static boolean inShuttlingZone5 = false;
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
    this.state = State.SHOOTING;
  }

  private void setState(State state) {
    this.state = state;
  }

  public Pose2d getTargetPose() {
    // Mirror pose if Blue Alliance
    if (alliance != null && alliance.equals(Alliance.Blue)) {
      return state.targetPose.rotateAround(
          Constants.FieldConstants.centerField, new Rotation2d(Math.PI));
    }

    return state.targetPose;
  }

  public State getState() {
    return this.state;
  }

  @Override
  public void periodic() {
    pose = Drive.getInstance().getPose();

    if (alliance == null && DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();
      if (alliance.equals(Alliance.Blue)) {
        updateBlueShuttleZones();
      }

      if (alliance.equals(Alliance.Red)) {
        updateRedShuttleZones();
      }
    }

    updateDecapitationZone();

    // Visualize zones in Ascope (turn Pose2d[] into trajectories to actually see rectangle)
    ZoneVisualization.logPoints(decapitationVisualization, "Decapitation Zone");

    // Update operation states based on if robot in zone
    OperationStates.inDecapitationZone =
        ZoneVisualization.isRobotInZone(decapitationVisualization, pose);

    // Update blue alliance
    if (DriverStation.getAlliance().isPresent() && alliance.equals(Alliance.Blue)) {
      logBlueZones();

      OperationStates.inShuttlingZone1 =
          ZoneVisualization.isRobotInZone(blueShuttlingZoneOneVisualization, pose);
      OperationStates.inShuttlingZone2 =
          ZoneVisualization.isRobotInZone(blueShuttlingZoneTwoVisualization, pose);
      OperationStates.inShuttlingZone3 =
          ZoneVisualization.isRobotInZone(blueShuttlingZoneThreeVisualization, pose);
      OperationStates.inShuttlingZone4 =
          ZoneVisualization.isRobotInZone(blueShuttlingZoneFourVisualization, pose);
      OperationStates.inShuttlingZone5 =
          ZoneVisualization.isRobotInZone(blueShuttlingZoneFiveVisualization, pose);
    }

    // Update red alliance
    if (DriverStation.getAlliance().isPresent() && alliance.equals(Alliance.Red)) {
      logRedZones();

      OperationStates.inShuttlingZone1 =
          ZoneVisualization.isRobotInZone(redShuttlingZoneOneVisualization, pose);
      OperationStates.inShuttlingZone2 =
          ZoneVisualization.isRobotInZone(redShuttlingZoneTwoVisualization, pose);
      OperationStates.inShuttlingZone3 =
          ZoneVisualization.isRobotInZone(redShuttlingZoneThreeVisualization, pose);
      OperationStates.inShuttlingZone4 =
          ZoneVisualization.isRobotInZone(redShuttlingZoneFourVisualization, pose);
      OperationStates.inShuttlingZone5 =
          ZoneVisualization.isRobotInZone(redShuttlingZoneFiveVisualization, pose);
    }

    // Set State
    if (OperationStates.inShuttlingZone1 || OperationStates.inShuttlingZone3) {
      setState(State.SHUTTLING_RIGHT_SIDE);
    } else if (OperationStates.inShuttlingZone2 || OperationStates.inShuttlingZone4) {
      setState(State.SHUTTLING_LEFT_SIDE);
    } else if (OperationStates.inShuttlingZone5) {
      setState(State.SHUTTLING_DEAD_ZONE);
    } else {
      setState(State.SHOOTING);
    }

    // Logging
    Logger.recordOutput(
        "State Manager/Operation States/In Decapitation Zone", OperationStates.inDecapitationZone);

    Logger.recordOutput(
        "State Manager/Operation States/In Zone 1", OperationStates.inShuttlingZone1);
    Logger.recordOutput(
        "State Manager/Operation States/In Zone 2", OperationStates.inShuttlingZone2);
    Logger.recordOutput(
        "State Manager/Operation States/In Zone 3", OperationStates.inShuttlingZone3);
    Logger.recordOutput(
        "State Manager/Operation States/In Zone 4", OperationStates.inShuttlingZone4);
    Logger.recordOutput(
        "State Manager/Operation States/In Zone 5", OperationStates.inShuttlingZone5);

    Logger.recordOutput("State Manager/State", state);
    Logger.recordOutput("State Manager/State Target", getTargetPose());
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

    decapitationVisualization = ZoneVisualization.getZone(decapitationZone);
  }

  public void updateBlueShuttleZones() {
    this.blueShuttlingZoneOne = Constants.FieldConstants.blueShuttleZoneOne;
    this.blueShuttlingZoneTwo = Constants.FieldConstants.blueShuttleZoneTwo;
    this.blueShuttlingZoneThree = Constants.FieldConstants.blueShuttleZoneThree;
    this.blueShuttlingZoneFour = Constants.FieldConstants.blueShuttleZoneFour;
    this.blueShuttlingZoneFive = Constants.FieldConstants.blueShuttleZoneFive;

    blueShuttlingZoneOneVisualization = ZoneVisualization.getZone(blueShuttlingZoneOne);
    blueShuttlingZoneTwoVisualization = ZoneVisualization.getZone(blueShuttlingZoneTwo);
    blueShuttlingZoneThreeVisualization = ZoneVisualization.getZone(blueShuttlingZoneThree);
    blueShuttlingZoneFourVisualization = ZoneVisualization.getZone(blueShuttlingZoneFour);
    blueShuttlingZoneFiveVisualization = ZoneVisualization.getZone(blueShuttlingZoneFive);
  }

  public void updateRedShuttleZones() {
    this.redShuttlingZoneOne = Constants.FieldConstants.redShuttleZoneOne;
    this.redShuttlingZoneTwo = Constants.FieldConstants.redShuttleZoneTwo;
    this.redShuttlingZoneThree = Constants.FieldConstants.redShuttleZoneThree;
    this.redShuttlingZoneFour = Constants.FieldConstants.redShuttleZoneFour;
    this.redShuttlingZoneFive = Constants.FieldConstants.redShuttleZoneFive;

    redShuttlingZoneOneVisualization = ZoneVisualization.getZone(redShuttlingZoneOne);
    redShuttlingZoneTwoVisualization = ZoneVisualization.getZone(redShuttlingZoneTwo);
    redShuttlingZoneThreeVisualization = ZoneVisualization.getZone(redShuttlingZoneThree);
    redShuttlingZoneFourVisualization = ZoneVisualization.getZone(redShuttlingZoneFour);
    redShuttlingZoneFiveVisualization = ZoneVisualization.getZone(redShuttlingZoneFive);
  }

  public void logRedZones() {
    ZoneVisualization.logPoints(redShuttlingZoneOneVisualization, "Shuttle Zone 1");
    ZoneVisualization.logPoints(redShuttlingZoneTwoVisualization, "Shuttle Zone 2");
    ZoneVisualization.logPoints(redShuttlingZoneThreeVisualization, "Shuttle Zone 3");
    ZoneVisualization.logPoints(redShuttlingZoneFourVisualization, "Shuttle Zone 4");
    ZoneVisualization.logPoints(redShuttlingZoneFiveVisualization, "Shuttle Zone 5");
  }

  public void logBlueZones() {
    ZoneVisualization.logPoints(blueShuttlingZoneOneVisualization, "Shuttle Zone 1");
    ZoneVisualization.logPoints(blueShuttlingZoneTwoVisualization, "Shuttle Zone 2");
    ZoneVisualization.logPoints(blueShuttlingZoneThreeVisualization, "Shuttle Zone 3");
    ZoneVisualization.logPoints(blueShuttlingZoneFourVisualization, "Shuttle Zone 4");
    ZoneVisualization.logPoints(blueShuttlingZoneFiveVisualization, "Shuttle Zone 5");
  }
}
