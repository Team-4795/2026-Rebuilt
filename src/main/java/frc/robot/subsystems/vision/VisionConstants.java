package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

public class VisionConstants {
  // all placeholders
  public static final double BORDER_MARGIN = 0.5;
  public static final double Z_MARGIN = 0.35;

  public static final double XY_SINGLE_STDEV = 0.08;
  public static final double XY_MULTIPLE_STDEV = 0.04;

  public static final String[] CAM_NAMES = {"Front Cam", "Left Cam", "Right Cam"};

  public static final Transform3d[] CAM_POSES =
      new Transform3d[] {
        // Front cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-16.25),
                Units.inchesToMeters(5.75),
                Units.inchesToMeters(11.5)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(0), Units.degreesToRadians(180))),

        // Side left cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0), Units.inchesToMeters(14.5), Units.inchesToMeters(4)),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-15),
                Units.degreesToRadians(90))),

        // Side right cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(0), Units.inchesToMeters(-14.5), Units.inchesToMeters(4)),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-15),
                Units.degreesToRadians(-90)))
      };

  // currently does not have 2026 field, check back when they update it
  public static AprilTagFieldLayout FIELD_LAYOUT;

  static {
    try {
      FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource("/frc/robot/2026AprilTag.json");
      FIELD_LAYOUT.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}
