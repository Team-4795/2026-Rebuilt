package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // all placeholders
  public static final double BORDER_MARGIN = 0.5;
  public static final double Z_MARGIN = 0.35;

  public static final double XY_SINGLE_STDEV = 0.08;
  public static final double XY_MULTIPLE_STDEV = 0.04;

  public static final String[] CAM_NAMES = {"Luma Right", "Back Cam", "Luma Left"};

  public static final Transform3d[] CAM_POSES =
      new Transform3d[] {
        // Luma Right
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-3.25),
                Units.inchesToMeters(-13.895),
                Units.inchesToMeters(8.75)),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-20),
                Units.degreesToRadians(-90))),

        // Back Cam
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-12.343),
                Units.inchesToMeters(-11),
                Units.inchesToMeters(8.750)),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-20),
                Units.degreesToRadians(180))),

        // Luma Left
        new Transform3d(
            new Translation3d( // Change Translation
                Units.inchesToMeters(0), Units.inchesToMeters(0), Units.inchesToMeters(0)),
            new Rotation3d( // Rotation should be good. Only touch if 3d pose looks weird
                Units.degreesToRadians(0), Units.degreesToRadians(-20), Units.degreesToRadians(90)))
      };

  public static AprilTagFieldLayout FIELD_LAYOUT;

  static {
    FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
    FIELD_LAYOUT.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
  }
}
