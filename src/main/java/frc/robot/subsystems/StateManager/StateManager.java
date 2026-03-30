package frc.robot.subsystems.StateManager;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeDeploy.IntakeDeploy;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHoodConstants;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Zone;
import org.littletonrobotics.junction.Logger;

public class StateManager extends SubsystemBase {
  private static StateManager instance;
  private State state;

  private MatchTimer timer;
  private boolean rumble;
  private final CommandXboxController driverController = OIConstants.driverController;

  // Corners of zone as an array
  private Translation2d[] decapitationZoneTranslation = new Translation2d[4];
  private Translation2d[] shuttlingZoneOneTranslation = Constants.FieldConstants.shuttleZoneOne;
  private Translation2d[] shuttlingZoneTwoTranslation = Constants.FieldConstants.shuttleZoneTwo;
  private Translation2d[] shuttlingZoneThreeTranslation = Constants.FieldConstants.shuttleZoneThree;
  private Translation2d[] shuttlingZoneFourTranslation = Constants.FieldConstants.shuttleZoneFour;
  private Translation2d[] shuttlingZoneFiveTranslation = Constants.FieldConstants.shuttleZoneFive;

  private Zone decapitationZone;
  private Zone shuttlingZoneOne;
  private Zone shuttlingZoneTwo;
  private Zone shuttlingZoneThree;
  private Zone shuttlingZoneFour;
  private Zone shuttlingZoneFive;

  private Alliance alliance;
  private Translation2d turretPose;

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
    this.decapitationZone = new Zone();
    this.shuttlingZoneOne = new Zone(shuttlingZoneOneTranslation);
    this.shuttlingZoneTwo = new Zone(shuttlingZoneTwoTranslation);
    this.shuttlingZoneThree = new Zone(shuttlingZoneThreeTranslation);
    this.shuttlingZoneFour = new Zone(shuttlingZoneFourTranslation);
    this.shuttlingZoneFive = new Zone(shuttlingZoneFiveTranslation);

    shuttlingZoneOne.logPoints("Shuttling Zone 1");
    shuttlingZoneTwo.logPoints("Shuttling Zone 2");
    shuttlingZoneThree.logPoints("Shuttling Zone 3");
    shuttlingZoneFour.logPoints("Shuttling Zone 4");
    shuttlingZoneFive.logPoints("Shuttling Zone 5");

    this.state = State.SHOOTING;

    timer = new MatchTimer();
  }

  private void setState(State state) {
    this.state = state;
  }

  public Pose2d getTargetPose() {
    // Mirror pose if Red Alliance
    if (alliance != null && alliance.equals(Alliance.Red)) {
      return state.targetPose.rotateAround(
          Constants.FieldConstants.centerField, new Rotation2d(Math.PI));
    }
    return state.targetPose;
  }

  public State getState() {
    return this.state;
  }

  public boolean canTurretMove() {
    return (TurretConstants.canMove
        && getState() != State.SHUTTLING_DEAD_ZONE
        && IntakeDeploy.getInstance().getPosition() < 0.25);
  }

  public boolean canHoodMove() {
    return !OperationStates.inDecapitationZone;
  }

  @Override
  public void periodic() {
    // Make shuttling zones red alliance if needed
    if (alliance == null && DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();
      timer.setAlliance(alliance);

      if (alliance.equals(Alliance.Red)) {
        shuttlingZoneOne.updateZone(toRedAlliance(shuttlingZoneOneTranslation));
        shuttlingZoneTwo.updateZone(toRedAlliance(shuttlingZoneTwoTranslation));
        shuttlingZoneThree.updateZone(toRedAlliance(shuttlingZoneThreeTranslation));
        shuttlingZoneFour.updateZone(toRedAlliance(shuttlingZoneFourTranslation));
        shuttlingZoneFive.updateZone(toRedAlliance(shuttlingZoneFiveTranslation));

        shuttlingZoneOne.logPoints("Shuttling Zone 1");
        shuttlingZoneTwo.logPoints("Shuttling Zone 2");
        shuttlingZoneThree.logPoints("Shuttling Zone 3");
        shuttlingZoneFour.logPoints("Shuttling Zone 4");
        shuttlingZoneFive.logPoints("Shuttling Zone 5");
      }
    }

    updateDecapitationZone();
    decapitationZone.logPoints("Decapitation Zone");

    turretPose = Turret.getInstance().getTurretPose();

    OperationStates.inDecapitationZone = decapitationZone.contains(turretPose);
    OperationStates.inShuttlingZone1 = shuttlingZoneOne.contains(turretPose);
    OperationStates.inShuttlingZone2 = shuttlingZoneTwo.contains(turretPose);
    OperationStates.inShuttlingZone3 = shuttlingZoneThree.contains(turretPose);
    OperationStates.inShuttlingZone4 = shuttlingZoneFour.contains(turretPose);
    OperationStates.inShuttlingZone5 = shuttlingZoneFive.contains(turretPose);

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

    // update match timer
    timer.updateAll();
    updateShiftRumble();

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
    Logger.recordOutput("State Manager/State Target Pose", getTargetPose());

    Logger.recordOutput(
        "State Manager/Is Ready/Shooter Ready", Shooter.getInstance().readyToShoot());
    Logger.recordOutput("State Manager/Is Ready/Turret Ready", Turret.getInstance().readyToShoot());
    Logger.recordOutput(
        "State Manager/Is Ready/Hood Ready", ShooterHood.getInstance().readyToShoot());

    Logger.recordOutput("State Manager/Timer/Current Shift", timer.getCurrentShift());
    Logger.recordOutput("State Manager/Timer/Next Shift", timer.getNextShift());
    Logger.recordOutput("State Manager/Timer/Our Next Shift", timer.getOurNextShift());
    Logger.recordOutput("State Manager/Timer/Who Won Auto?", timer.getAutoWinningAlliance());
    Logger.recordOutput("State Manager/Timer/Time Until Next Shift", timer.getTimeToShift());
    Logger.recordOutput("State Manager/Timer/Time Until Our Shift", timer.getTimeToOurShift());
    Logger.recordOutput("State Manager/Timer/Rumble Controller", rumble);

    Logger.recordOutput("State Manager/Alliance", timer.getOurAlliance());
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
        0.5 + ShooterHoodConstants.boxXMultiplier * Math.abs(fieldRelative.vxMetersPerSecond);
    double boxYDim = FieldConstants.trenchWidth;

    Translation2d topLeft = new Translation2d(closest.getX() - boxXDim, closest.getY() + boxYDim);
    Translation2d topRight = new Translation2d(closest.getX() + boxXDim, closest.getY() + boxYDim);
    Translation2d botLeft = new Translation2d(closest.getX() - boxXDim, closest.getY() - boxYDim);
    Translation2d botRight = new Translation2d(closest.getX() + boxXDim, closest.getY() - boxYDim);

    decapitationZoneTranslation[0] = topLeft;
    decapitationZoneTranslation[1] = topRight;
    decapitationZoneTranslation[2] = botLeft;
    decapitationZoneTranslation[3] = botRight;

    decapitationZone.updateZone(decapitationZoneTranslation);
  }

  public void startMatchTimer() {
    timer.startTeleop();
  }

  // TODO finish/test controller rumble
  private void updateShiftRumble() {
    double t = timer.getTimeToShift();
    if (t < 3.0) {
      rumble = true;
      driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      rumble = false;
      driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  private Translation2d[] toRedAlliance(Translation2d[] blueAllianceTranslation) {
    Translation2d[] redAllianceTranslation = new Translation2d[4];
    for (int i = 0; i < blueAllianceTranslation.length; i++) {
      redAllianceTranslation[i] =
          blueAllianceTranslation[i].rotateAround(
              Constants.FieldConstants.centerField, new Rotation2d(Math.PI));
    }
    return redAllianceTranslation;
  }
}
