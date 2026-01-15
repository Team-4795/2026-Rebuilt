package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  // all placeholders
  public static final double BORDER_MARGIN = 0.5;
  public static final double Z_MARGIN = 0.35;

  public static final double XY_SINGLE_STDEV = 0.08;
  public static final double XY_MULTIPLE_STDEV = 0.04;

  public static final String[] CAM_NAMES = {"mac and cheese"};

  public static final Transform3d[] CAM_POSES = {new Transform3d()};

  // currently does not have 2026 field, check back when they update it
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
}
