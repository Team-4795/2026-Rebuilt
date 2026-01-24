package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.Turret.Turret;
import frc.robot.subsystems.Turret.TurretConstants;

public class TurretCalculator {
  Pose2d robotPose;
  Translation2d hub;
  Translation2d turretOffsetPose;

  public TurretCalculator(Pose2d robotPosition, Turret turret) {
    turretOffsetPose = TurretConstants.OFFSET;
    Translation2d turretLocation =
        robotPose.getTranslation().plus(turretOffsetPose.rotateBy(robotPose.getRotation()));
    Translation2d hub = Constants.hub;
  }
}
