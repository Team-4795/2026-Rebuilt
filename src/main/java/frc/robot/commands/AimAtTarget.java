package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.StateManager.State;
import frc.robot.subsystems.StateManager.StateManager;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AimAtTarget extends Command {
  private final Turret turret;
  private final Drive drive;
  private final StateManager stateManager;
  Pose2d robotPose;
  Pose2d targetPose;
  Translation2d turretOffsetPose = TurretConstants.OFFSET;
  Translation2d turretPose;
  double deltaX = 0;
  double deltaY = 0;
  double desiredRot;
  double turretAngle;

  public AimAtTarget(Drive drive, Turret turret, StateManager stateManager) {
    this.drive = drive;
    this.turret = turret;
    this.stateManager = stateManager;
    addRequirements(turret);
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

    turretAngle =
        (Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians()) / (2 * Math.PI);
    turretAngle -= TurretConstants.angleOffset;
    desiredRot = (((turretAngle % 1.0) + 1.0) % 1.0);

    if (StateManager.getInstance().getState() != State.SHUTTLING_DEAD_ZONE) {
      turret.setGoal(desiredRot);
    }

    Logger.recordOutput("Aim At Hub/Hub Pose", targetPose);
    Logger.recordOutput("Aim At Hub/Desired Rotation", desiredRot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
