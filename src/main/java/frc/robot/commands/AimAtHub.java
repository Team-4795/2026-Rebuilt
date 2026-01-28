package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class AimAtHub extends Command {
  private final Turret turret;
  private final Drive drive;
  Pose2d robotPose;
  Translation2d hub = FieldConstants.Hub.innerCenterPoint.toTranslation2d();
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
  public void initialize() {
    System.out.println(
        "Aim is aiming!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  @Override
  public void execute() {
    System.out.println("Aim is executing");
    robotPose = drive.getPose();
    turretPose =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));

    deltaX = hub.getX() - turretPose.getX();
    deltaY = hub.getY() - turretPose.getY();

    turretAngle =
        (Math.atan2(deltaY, deltaX) - robotPose.getRotation().getRadians()) / (2 * Math.PI);
    turretAngle = -turretAngle - TurretConstants.angleOffset;
    desiredRot = (((turretAngle % 1.0) + 1.0) % 1.0);
    turret.setGoal(desiredRot);
    System.err.println("angle is" + desiredRot);

    Logger.recordOutput("Aim At Hub/Hub Pose", hub);
    Logger.recordOutput("Aim At Hub/Desired Rotation", desiredRot);
  }

  @Override
  public boolean isFinished() {
    System.out.println("aim is done");
    return false;
  }
}
