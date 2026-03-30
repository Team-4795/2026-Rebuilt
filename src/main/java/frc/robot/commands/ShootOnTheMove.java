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
import frc.robot.subsystems.ShooterHood.ShooterHoodIOReal;
import frc.robot.subsystems.StateManager.State;
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.StateManager.StateManager.OperationStates;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class ShootOnTheMove extends Command {
  private final Turret turret;
  private final Shooter shooter;
  private final ShooterHood hood;
  private final Drive drive;
  private final StateManager stateManager;
  Pose2d robotPose;
  Pose2d hub = StateManager.getInstance().getTargetPose();
  Pose2d targetPose;
  Pose2d offsettedTarget;
  Translation2d turretOffsetPose = TurretConstants.OFFSET;
  Translation2d turretPose;
  double deltaX = 0;
  double deltaY = 0;
  double velocityXOffset = 0;
  double velocityYOffset = 0;
  double velocityOmega = 0; // needs to be in rotations
  double omegaXOffset = 0;
  double omegaYOffset = 0;
  double desiredRot = 0;
  double turretAngle = 0;
  Translation2d velocityVector;
  double rotationOffset = TurretConstants.angleOffset; // zeroing offset
  double tAir = 1; // calculate this with utility class or interpolating tree
  double tLat; // time for indexer to actually shoot out a ball (latency)
  ChassisSpeeds fieldRelative;
  double iterativeDistance = 0;
  double distance;
  double dampener;

  public static LoggedTunableNumber shooterRPS = new LoggedTunableNumber("SOTM/Shooter RPS", 60);

  public ShootOnTheMove(
      Drive drive, Turret turret, Shooter shooter, ShooterHood hood, StateManager manager) {
    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
    this.stateManager = manager;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void initialize() {
    targetPose = stateManager.getTargetPose();
  }

  @Override
  public void execute() {
    dampener = 1;

    robotPose = drive.getPose();
    targetPose = stateManager.getTargetPose();
    turretPose =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));

    fieldRelative =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), robotPose.getRotation());

    velocityOmega = drive.getChassisSpeeds().omegaRadiansPerSecond;

    iterativeDistance = turretPose.getDistance(targetPose.getTranslation());

    if (StateManager.getInstance().getState() == State.SHOOTING) {
      for (int i = 0; i < 20; i++) {
        tAir = Constants.InterpolatingTree.tAirMap.get(iterativeDistance);

        velocityXOffset = fieldRelative.vxMetersPerSecond * tAir * dampener;
        velocityYOffset = fieldRelative.vyMetersPerSecond * tAir * dampener;

        omegaXOffset =
            -velocityOmega * turretOffsetPose.rotateBy(robotPose.getRotation()).getY() * tAir;
        omegaYOffset =
            velocityOmega * turretOffsetPose.rotateBy(robotPose.getRotation()).getX() * tAir;

        offsettedTarget =
            new Pose2d(
                targetPose.getX() - velocityXOffset - omegaXOffset,
                targetPose.getY() - velocityYOffset - omegaYOffset,
                Rotation2d.kZero);

        iterativeDistance = offsettedTarget.getTranslation().getDistance(turretPose);
      }
    } else {
      tAir = 1.5;

      velocityXOffset = fieldRelative.vxMetersPerSecond * tAir * dampener;
      velocityYOffset = fieldRelative.vyMetersPerSecond * tAir * dampener;

      omegaXOffset =
          -velocityOmega * turretOffsetPose.rotateBy(robotPose.getRotation()).getY() * tAir;
      omegaYOffset =
          velocityOmega * turretOffsetPose.rotateBy(robotPose.getRotation()).getX() * tAir;

      offsettedTarget =
          new Pose2d(
              targetPose.getX() - velocityXOffset - omegaXOffset,
              targetPose.getY() - velocityYOffset - omegaYOffset,
              Rotation2d.kZero);
    }

    distance = offsettedTarget.getTranslation().getDistance(turretPose);

    deltaX = offsettedTarget.getX() - turretPose.getX();
    deltaY = offsettedTarget.getY() - turretPose.getY();

    turretAngle =
        (Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians()) / (2 * Math.PI);
    turretAngle -= TurretConstants.angleOffset;
    desiredRot = (((turretAngle % 1.0) + 1.0) % 1.0);

    turret.setGoal(desiredRot);

    if (stateManager.getState() == State.SHOOTING && !OperationStates.inDecapitationZone) {
      // shooter.setVelocityRPS(ShooterConstants.shooterVelocityHubMap.get(distance));
      // hood.setGoal(ShooterHoodConstants.shooterHoodHubMap.get(distance));
      shooter.setVelocityRPS(shooterRPS.getAsDouble());
      hood.setGoal(ShooterHoodIOReal.hoodAngle.getAsDouble());
    } else if (!OperationStates.inDecapitationZone) {
      shooter.setVelocityRPS(ShooterConstants.shooterVelocityShuttlingMap.get(distance));
      hood.setGoal(ShooterHoodConstants.shooterHoodShuttlingMap.get(distance));
    }

    Logger.recordOutput("Shoot on move At Hub/Hub Pose", hub);
    Logger.recordOutput("Shoot on move At Hub/Desired Hub", offsettedTarget);

    Logger.recordOutput("Shoot on move At Hub/Distance to Desire Hub", distance);

    Logger.recordOutput("Shoot on move At Hub/Desired Rotation", desiredRot);
    Logger.recordOutput("Shoot on move At Hub/ XOffset", deltaX);
    Logger.recordOutput("Shoot on move At Hub/ YOffset", deltaY);
    Logger.recordOutput("Shoot on move At Hub/ OmegaXOffset", omegaXOffset);
    Logger.recordOutput("Shoot on move At Hub/ OmegaYOffset", omegaYOffset);
    Logger.recordOutput("Shoot on move At Hub/ LinearXOffset", velocityXOffset);
    Logger.recordOutput("Shoot on move At Hub/ LinearYOffset", velocityYOffset);
    Logger.recordOutput("Shoot on move At Hub/tAir", tAir);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
