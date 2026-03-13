package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.ShooterHood.ShooterHood;
import frc.robot.subsystems.ShooterHood.ShooterHoodConstants;
import frc.robot.subsystems.StateManager.State;
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AimAtTarget extends Command {
  private final Turret turret;
  private final Shooter shooter;
  private final ShooterHood hood;
  private final Drive drive;
  private final StateManager stateManager;
  Pose2d robotPose;
  Pose2d targetPose;
  Translation2d turretOffsetPose = TurretConstants.OFFSET;
  Translation2d turretPose;
  double deltaX = 0;
  double deltaY = 0;
  double distance;
  double desiredRot;
  double turretAngle;

  public AimAtTarget(
      Drive drive, Turret turret, Shooter shooter, ShooterHood hood, StateManager stateManager) {
    this.drive = drive;
    this.turret = turret;
    this.shooter = shooter;
    this.hood = hood;
    this.stateManager = stateManager;
    addRequirements(turret, shooter, hood);
  }

  @Override
  public void initialize() {
    targetPose = stateManager.getTargetPose();
  }

  @Override
  public void execute() {
    targetPose = stateManager.getTargetPose();
    robotPose = drive.getPose();

    turretPose =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));

    deltaX = targetPose.getX() - turretPose.getX();
    deltaY = targetPose.getY() - turretPose.getY();

    distance = targetPose.getTranslation().getDistance(turretPose);

    turretAngle =
        (Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians()) / (2 * Math.PI);
    turretAngle -= TurretConstants.angleOffset;
    desiredRot = (((turretAngle % 1.0) + 1.0) % 1.0);

    turret.setGoal(desiredRot);

    if (stateManager.getState() == State.SHOOTING) {
      shooter.setVelocityRPS(ShooterConstants.shooterVelocityHubMap.get(distance));
      hood.setGoal(ShooterHoodConstants.shooterHoodHubMap.get(distance));
    } else {
      shooter.setVelocityRPS(ShooterConstants.shooterVelocityShuttlingMap.get(distance));
      hood.setGoal(ShooterHoodConstants.shooterHoodShuttlingMap.get(distance));
    }

    Logger.recordOutput("Aim At Hub/Hub Pose", targetPose);
    Logger.recordOutput("Aim At Hub/Desired Rotation", desiredRot);
    Logger.recordOutput("Aim At Hub/Distance", distance);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
