package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHoodConstants;
import frc.robot.subsystems.StateManager.State;
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class NewSOTM extends Command {
  private final Turret turret;
  private final Shooter shooter;
  private final ShooterHood hood;
  private final Drive drive;
  private final StateManager stateManager;
  private Translation2d targetPos;
  private Translation2d offsetTargetPos;
  private Pose2d robotPose;
  private Translation2d turretPos;
  private ChassisSpeeds robotSpeeds;
  private double vxOffset;
  private double vyOffset;
  private double omegaXOffset;
  private double omegaYOffset;
  private double tAir;
  private double distToTargetOriginal;
  private double distToTargetOffset;
  private double turretToTargetX;
  private double turretToTargetY;
  private double turretAngleToTarget;
  private double turretGoalRots;

  // dampeners for velocity offsets, tune as needed
  public static LoggedTunableNumber omegaDampener =
      new LoggedTunableNumber("New SOTM/Omega Dampener", 0.05);
  public static LoggedTunableNumber xDampener = new LoggedTunableNumber("New SOTM/X Dampener", 0.7);
  public static LoggedTunableNumber yDampener =
      new LoggedTunableNumber("New SOTM/Y Dampener", 0.85);

  public NewSOTM(
      Drive drive, Turret turret, Shooter shooter, ShooterHood hood, StateManager manager) {
    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
    this.stateManager = manager;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    targetPos = stateManager.getTargetPose().getTranslation();
    robotSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation());
    robotPose = drive.getPose();
    turretPos =
        robotPose
            .getTranslation()
            .plus(TurretConstants.OFFSET)
            .rotateAround(robotPose.getTranslation(), robotPose.getRotation());
    distToTargetOriginal = turretPos.getDistance(targetPos);

    // get time in air from map
    tAir = Constants.InterpolatingTree.tAirMap.get(distToTargetOriginal);

    // calculate x and y position offsets by multiplying velocity with tAir
    vxOffset = robotSpeeds.vxMetersPerSecond * tAir;
    vyOffset = robotSpeeds.vyMetersPerSecond * tAir;

    // similar for rotation offset
    omegaXOffset =
        robotSpeeds.omegaRadiansPerSecond
            * turretPos.rotateBy(robotPose.getRotation()).getX()
            * tAir
            * omegaDampener.getAsDouble();
    omegaYOffset =
        robotSpeeds.omegaRadiansPerSecond
            * turretPos.rotateBy(robotPose.getRotation()).getY()
            * tAir
            * omegaDampener.getAsDouble();

    // translation2d representing target after offsets have been applied
    offsetTargetPos =
        new Translation2d(
            targetPos.getX() - vxOffset - omegaXOffset, targetPos.getY() - vyOffset - omegaYOffset);

    // find distance from offset target, need for shooter hood and shooter maps
    distToTargetOffset = offsetTargetPos.getDistance(turretPos);

    // find difference between x and y coords of turret and target to calculate angle
    turretToTargetX = offsetTargetPos.getX() - turretPos.getX();
    turretToTargetY = offsetTargetPos.getY() - turretPos.getY();
    turretAngleToTarget = Math.atan2(turretToTargetY, turretToTargetX);

    // calculate turret goal rotations, need to convert from field-relative angle to turret-relative
    turretGoalRots = (turretAngleToTarget - robotPose.getRotation().getRadians()) / (2 * Math.PI);
    turretGoalRots -= TurretConstants.angleOffset;
    turretGoalRots = (((turretGoalRots % 1.0) + 1.0) % 1.0); // prevents weird stuff with wrapping

    turret.setGoal(turretGoalRots); // set turret goal!!

    // different shooter hood/shooter maps depending on whether robot is shuttling or shooting at
    // hub
    if (stateManager.getState() == State.SHOOTING) {
      hood.setGoal(ShooterHoodConstants.shooterHoodHubMap.get(distToTargetOffset));
      shooter.setVelocityRPS(ShooterConstants.shooterVelocityHubMap.get(distToTargetOffset));
    } else if (stateManager.getState() != State.SHUTTLING_DEAD_ZONE) {
      hood.setGoal(ShooterHoodConstants.shooterHoodShuttlingMap.get(distToTargetOffset));
      shooter.setVelocityRPS(ShooterConstants.shooterVelocityShuttlingMap.get(distToTargetOffset));
    }

    // logging stuff
    Logger.recordOutput("New SOTM/Original target", new Pose2d(targetPos, Rotation2d.kZero));
    Logger.recordOutput("New SOTM/Offset target", new Pose2d(offsetTargetPos, Rotation2d.kZero));
    Logger.recordOutput("New SOTM/Dist to original target", distToTargetOriginal);
    Logger.recordOutput("New SOTM/Dist to offset target", distToTargetOffset);
    Logger.recordOutput("New SOTM/Turret angle to target", turretAngleToTarget);
    Logger.recordOutput("New SOTM/Turret Goal", turretGoalRots);
    Logger.recordOutput("New SOTM/vxOffset", vxOffset);
    Logger.recordOutput("New SOTM/vyOffset", vyOffset);
    Logger.recordOutput("New SOTM/omegaXOffset", omegaXOffset);
    Logger.recordOutput("New SOTM/omegaYOffset", omegaYOffset);
  }
}
