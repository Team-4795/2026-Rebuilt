package frc.robot.subsystems.StateManager;

import edu.wpi.first.math.geometry.Pose2d;
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
import org.littletonrobotics.junction.Logger;

public class StateManager extends SubsystemBase {
  private static StateManager instance;

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
  private ZoneVisualization decapitationVisualization = new ZoneVisualization();

  private ZoneVisualization blueShuttlingZoneOneVisualization = new ZoneVisualization();
  private ZoneVisualization blueShuttlingZoneTwoVisualization = new ZoneVisualization();
  private ZoneVisualization blueShuttlingZoneThreeVisualization = new ZoneVisualization();
  private ZoneVisualization blueShuttlingZoneFourVisualization = new ZoneVisualization();

  private ZoneVisualization redShuttlingZoneOneVisualization = new ZoneVisualization();
  private ZoneVisualization redShuttlingZoneTwoVisualization = new ZoneVisualization();
  private ZoneVisualization redShuttlingZoneThreeVisualization = new ZoneVisualization();
  private ZoneVisualization redShuttlingZoneFourVisualization = new ZoneVisualization();

  private Alliance alliance;

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
    if (alliance == null && DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();

      if (alliance.equals(Alliance.Blue)) {
        this.blueShuttlingZoneOne = Constants.FieldConstants.blueShuttleZoneOne;
        this.blueShuttlingZoneTwo = Constants.FieldConstants.blueShuttleZoneTwo;
        this.blueShuttlingZoneThree = Constants.FieldConstants.blueShuttleZoneThree;
        this.blueShuttlingZoneFour = Constants.FieldConstants.blueShuttleZoneFour;

        blueShuttlingZoneOneVisualization.updateZone(blueShuttlingZoneOne);
        blueShuttlingZoneTwoVisualization.updateZone(blueShuttlingZoneTwo);
        blueShuttlingZoneThreeVisualization.updateZone(blueShuttlingZoneThree);
        blueShuttlingZoneFourVisualization.updateZone(blueShuttlingZoneFour);
      }

      if (alliance.equals(Alliance.Red)) {
        this.redShuttlingZoneOne = Constants.FieldConstants.redShuttleZoneOne;
        this.redShuttlingZoneTwo = Constants.FieldConstants.redShuttleZoneTwo;
        this.redShuttlingZoneThree = Constants.FieldConstants.redShuttleZoneThree;
        this.redShuttlingZoneFour = Constants.FieldConstants.redShuttleZoneFour;

        redShuttlingZoneOneVisualization.updateZone(redShuttlingZoneOne);
        redShuttlingZoneTwoVisualization.updateZone(redShuttlingZoneTwo);
        redShuttlingZoneThreeVisualization.updateZone(redShuttlingZoneThree);
        redShuttlingZoneFourVisualization.updateZone(redShuttlingZoneFour);
      }
    }

    updateDecapitationZone();

    // Visualize zones in Ascope (turn Pose2d[] into trajectories to actually see rectangle)
    decapitationVisualization.logPoints("Decapitation Zone");

    if (DriverStation.getAlliance().isPresent() && alliance.equals(Alliance.Blue)) {
      blueShuttlingZoneOneVisualization.logPoints("Shuttle Zone 1");
      blueShuttlingZoneTwoVisualization.logPoints("Shutttle Zone 2");
      blueShuttlingZoneThreeVisualization.logPoints("Shutttle Zone 3");
      blueShuttlingZoneFourVisualization.logPoints("Shutttle Zone 4");
    }

    if (DriverStation.getAlliance().isPresent() && alliance.equals(Alliance.Red)) {
      redShuttlingZoneOneVisualization.logPoints("Red Shuttle Zone 1");
      redShuttlingZoneTwoVisualization.logPoints("Red Shuttle Zone 2");
      redShuttlingZoneThreeVisualization.logPoints("Red Shuttle Zone 3");
      redShuttlingZoneFourVisualization.logPoints("Red Shuttle Zone 4");
    }

    // Update operation states based on if robot in zone
    OperationStates.inDecapitationZone =
        decapitationVisualization.isRobotInZone(Drive.getInstance().getPose());

    if (DriverStation.getAlliance().isPresent() && alliance.equals(Alliance.Blue)) {
      OperationStates.inShuttlingZone1 =
          blueShuttlingZoneOneVisualization.isRobotInZone(Drive.getInstance().getPose());
      OperationStates.inShuttlingZone2 =
          blueShuttlingZoneTwoVisualization.isRobotInZone(Drive.getInstance().getPose());
      OperationStates.inShuttlingZone3 =
          blueShuttlingZoneThreeVisualization.isRobotInZone(Drive.getInstance().getPose());
      OperationStates.inShuttlingZone4 =
          blueShuttlingZoneFourVisualization.isRobotInZone(Drive.getInstance().getPose());
    }

    if (DriverStation.getAlliance().isPresent() && alliance.equals(Alliance.Red)) {
      OperationStates.inShuttlingZone1 =
          redShuttlingZoneOneVisualization.isRobotInZone(Drive.getInstance().getPose());
      OperationStates.inShuttlingZone2 =
          redShuttlingZoneTwoVisualization.isRobotInZone(Drive.getInstance().getPose());
      OperationStates.inShuttlingZone3 =
          redShuttlingZoneThreeVisualization.isRobotInZone(Drive.getInstance().getPose());
      OperationStates.inShuttlingZone4 =
          redShuttlingZoneFourVisualization.isRobotInZone(Drive.getInstance().getPose());
    }

    Logger.recordOutput(
        "Operation States/In Decapitation Zone", OperationStates.inDecapitationZone);

    Logger.recordOutput("Operation States/In Zone One", OperationStates.inShuttlingZone1);
    Logger.recordOutput("Operation States/In Zone Two", OperationStates.inShuttlingZone2);
    Logger.recordOutput("Operation States/In Zone Three", OperationStates.inShuttlingZone3);
    Logger.recordOutput("Operation States/In Zone Four", OperationStates.inShuttlingZone4);
  }
}
