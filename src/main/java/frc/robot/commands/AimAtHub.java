package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AimAtHub extends Command {
  private final Turret turret;
  private final Drive drive;
  Pose2d robotPose;
  Translation2d hub = Constants.FieldConstants.redHub;
  Translation2d turretOffsetPose = TurretConstants.OFFSET;
  Translation2d turretPose;
  double deltaX = 0;
  double deltaY = 0;
  double desiredRot;
  double turretAngle;

  public AimAtHub(Drive drive, Turret turret) {
    this.drive = drive;
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    robotPose = drive.getPose();
    turretPose =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));

    deltaX = hub.getX() - turretPose.getX();
    deltaY = hub.getY() - turretPose.getY();

    turretAngle =
        (Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians()) / (2 * Math.PI);
    turretAngle -= TurretConstants.angleOffset;
    desiredRot = (((turretAngle % 1.0) + 1.0) % 1.0);
    turret.setGoal(desiredRot);

    Logger.recordOutput("Aim At Hub/Hub Pose", hub);
    Logger.recordOutput("Aim At Hub/Desired Rotation", desiredRot);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
